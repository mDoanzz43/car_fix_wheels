#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import threading
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
import os

class LaserScanRegionFilterWithCamera:
    def __init__(self):
        rospy.init_node("laser_scan_region_filter_with_camera_node", anonymous=True)

        # ==== Định nghĩa vùng cấm (vùng giả định có người đứng im) để lọc laser scan ====
        self.forbidden_angle_min = -0.8  # Góc nhỏ nhất của vùng cấm (radian), ví dụ: -30 độ
        self.forbidden_angle_max = 0.8   # Góc lớn nhất của vùng cấm (radian), ví dụ: 30 độ
        self.max_distance = 5.0  # Chỉ lọc các điểm trong khoảng cách 5 mét

        # ==== YOLO Configuration ====
        self.conf_threshold = 0.5
        self.nms_threshold = 0.3
        self.input_size = (416, 416)
        self.classes = ['person']
        self.bridge = CvBridge()
        # ==== Load YOLOv3-Tiny Model ====
        cfg_path = "/home/doan/ck2_ws/src/car_fix_wheels/models/yolov3-tiny.cfg"
        weights_path = "/home/doan/ck2_ws/src/car_fix_wheels/models/yolov3-tiny.weights"

        if not os.path.exists(cfg_path) or not os.path.exists(weights_path):
            rospy.logerr("❌ Không tìm thấy file mô hình YOLO!")
            raise FileNotFoundError("Missing YOLOv3-Tiny config or weights")

        self.net = cv2.dnn.readNetFromDarknet(cfg_path, weights_path)
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

        layer_names = self.net.getLayerNames()
        unconnected_layers = self.net.getUnconnectedOutLayers()
        if len(unconnected_layers.shape) == 1:
            self.output_layers = [layer_names[i - 1] for i in unconnected_layers]
        else:
            self.output_layers = [layer_names[i[0] - 1] for i in unconnected_layers]

        # ==== Frame Processing for Camera Display ====
        self.display_frame = None
        self.new_frame_available = False
        self.lock = threading.Lock()
        self.frame_skip = 2
        self.frame_counter = 0

        # ==== ROS Subscribers and Publishers ====
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback, queue_size=1)
        self.image_sub = rospy.Subscriber("/image_raw", Image, self.image_callback, queue_size=1, buff_size=2**18)
        self.scan_pub = rospy.Publisher("/scan_filtered", LaserScan, queue_size=1)

        # Start display thread
        threading.Thread(target=self.display_loop, daemon=True).start()

        rospy.loginfo("✅ Laser Scan Region Filter with Camera Display (YOLO Detection) is running...")

    def merge_boxes(self, boxes, confidences, class_ids):
        if not boxes:
            return [], [], []

        merged_boxes = []
        merged_confidences = []
        merged_class_ids = []

        indices = sorted(range(len(confidences)), key=lambda i: confidences[i], reverse=True)

        while indices:
            i = indices[0]
            x1, y1, w1, h1 = boxes[i]
            conf1 = confidences[i]
            class1 = class_ids[i]
            merged_box = [x1, y1, x1 + w1, y1 + h1]
            indices_to_remove = [i]

            for j in indices[1:]:
                x2, y2, w2, h2 = boxes[j]
                box2 = [x2, y2, x2 + w2, y2 + h2]
                x_left = max(merged_box[0], box2[0])
                y_top = max(merged_box[1], box2[1])
                x_right = min(merged_box[2], box2[2])
                y_bottom = min(merged_box[3], box2[3])
                intersection = max(0, x_right - x_left) * max(0, y_bottom - y_top)
                area1 = (merged_box[2] - merged_box[0]) * (merged_box[3] - merged_box[1])
                area2 = (box2[2] - box2[0]) * (box2[3] - box2[1])
                union = area1 + area2 - intersection
                iou = intersection / union if union > 0 else 0

                if iou > 0.3 or (abs(merged_box[0] - box2[0]) < w1 * 0.5 and abs(merged_box[1] - box2[1]) < h1 * 0.5):
                    merged_box[0] = min(merged_box[0], box2[0])
                    merged_box[1] = min(merged_box[1], box2[1])
                    merged_box[2] = max(merged_box[2], box2[2])
                    merged_box[3] = max(merged_box[3], box2[3])
                    indices_to_remove.append(j)

            merged_x = merged_box[0]
            merged_y = merged_box[1]
            merged_w = merged_box[2] - merged_box[0]
            merged_h = merged_box[3] - merged_box[1]

            merged_boxes.append([merged_x, merged_y, merged_w, merged_h])
            merged_confidences.append(conf1)
            merged_class_ids.append(class1)

            indices = [idx for idx in indices if idx not in indices_to_remove]

        return merged_boxes, merged_confidences, merged_class_ids

    def detect(self, image):
        height, width = image.shape[:2]
        blob = cv2.dnn.blobFromImage(image, 1/255.0, self.input_size, swapRB=True, crop=False)
        self.net.setInput(blob)
        outputs = self.net.forward(self.output_layers)

        boxes, confidences, class_ids = [], [], []

        for output in outputs:
            for det in output:
                scores = det[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > self.conf_threshold and class_id == 0:
                    center_x = int(det[0] * width)
                    center_y = int(det[1] * height)
                    w = int(det[2] * width)
                    h = int(det[3] * height)
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)
                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)

        return boxes, confidences, class_ids

    def scan_callback(self, msg):
        # Tạo bản sao của laser scan để chỉnh sửa
        filtered_scan = LaserScan()
        filtered_scan.header = msg.header
        filtered_scan.angle_min = msg.angle_min
        filtered_scan.angle_max = msg.angle_max
        filtered_scan.angle_increment = msg.angle_increment
        filtered_scan.time_increment = msg.time_increment
        filtered_scan.scan_time = msg.scan_time
        filtered_scan.range_min = msg.range_min
        filtered_scan.range_max = msg.range_max
        filtered_scan.ranges = list(msg.ranges)
        filtered_scan.intensities = list(msg.intensities)

        # Duyệt qua từng tia laser để kiểm tra và lọc
        angle = msg.angle_min
        for i, r in enumerate(msg.ranges):
            if np.isinf(r) or np.isnan(r):
                angle += msg.angle_increment
                continue

            # Kiểm tra nếu tia laser nằm trong vùng cấm
            if (self.forbidden_angle_min <= angle <= self.forbidden_angle_max) and (r <= self.max_distance):
                filtered_scan.ranges[i] = float('inf')  # Loại bỏ tia laser trong vùng cấm

            angle += msg.angle_increment

        # Xuất bản laser scan đã được lọc
        self.scan_pub.publish(filtered_scan)

    def image_callback(self, msg):
        self.frame_counter += 1
        if self.frame_counter % self.frame_skip != 0:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr("❌ Lỗi chuyển ảnh: %s", str(e))
            return

        frame = cv2.resize(frame, (600, 512), interpolation=cv2.INTER_LINEAR)
        frame = cv2.convertScaleAbs(frame, alpha=1.5, beta=30)

        # Phát hiện người bằng YOLO
        boxes, confidences, class_ids = self.detect(frame)
        indices = cv2.dnn.NMSBoxes(boxes, confidences, self.conf_threshold, self.nms_threshold)

        if len(indices) > 0:
            indices = indices.flatten()
            nms_boxes = [boxes[i] for i in indices]
            nms_confidences = [confidences[i] for i in indices]
            nms_class_ids = [class_ids[i] for i in indices]

            merged_boxes, merged_confidences, merged_class_ids = self.merge_boxes(nms_boxes, nms_confidences, nms_class_ids)

            for i in range(len(merged_boxes)):
                x, y, w, h = merged_boxes[i]
                label = f"{self.classes[merged_class_ids[i]]} {merged_confidences[i]:.2f}"
                # Vẽ bounding box màu xanh dương (BGR: 255, 0, 0)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
                cv2.putText(frame, label, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

        with self.lock:
            self.display_frame = frame
            self.new_frame_available = True

    def display_loop(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.new_frame_available:
                with self.lock:
                    if self.display_frame is None:
                        continue
                    frame = self.display_frame.copy()
                    self.new_frame_available = False
                cv2.imshow("Camera with Person Detection", frame)
            if cv2.waitKey(1) == 27:  # Nhấn ESC để thoát
                rospy.signal_shutdown("ESC pressed")
            rate.sleep()

if __name__ == '__main__':
    try:
        filter_node = LaserScanRegionFilterWithCamera()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()