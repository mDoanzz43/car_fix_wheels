#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import threading
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os

class YOLOPersonDetector:
    def __init__(self):
        rospy.init_node("yolo_person_detector_node", anonymous=True)

        # ==== Configuration ====
        self.conf_threshold = 0.5  # Lowered to catch more potential full-body detections
        self.nms_threshold = 0.3   # Lowered to allow more aggressive merging of overlapping boxes
        self.input_size = (416, 416)  # Increased input size for better detection of smaller features
        self.classes = ['person']
        self.bridge = CvBridge()

        # ==== Load YOLOv3-Tiny Model ====
        model_dir = "/home/dat/catkin_ws/src/car_fix_wheels/scripts"
        cfg_path = os.path.join(model_dir, "yolov3-tiny.cfg")
        weights_path = os.path.join(model_dir, "yolov3-tiny.weights")

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

        # ==== Frame Processing ====
        self.display_frame = None
        self.new_frame_available = False
        self.lock = threading.Lock()
        self.frame_skip = 2
        self.frame_counter = 0

        # ==== ROS Subscriber ====
        rospy.Subscriber("/image_raw", Image, self.image_callback, queue_size=1, buff_size=2**18)
        threading.Thread(target=self.display_loop, daemon=True).start()

        rospy.loginfo("✅ YOLO Person Detector đã sẵn sàng!")

    def merge_boxes(self, boxes, confidences, class_ids):
        if not boxes:
            return [], [], []

        merged_boxes = []
        merged_confidences = []
        merged_class_ids = []

        # Sort by confidence to prioritize higher confidence detections
        indices = sorted(range(len(confidences)), key=lambda i: confidences[i], reverse=True)

        while indices:
            i = indices[0]
            x1, y1, w1, h1 = boxes[i]
            conf1 = confidences[i]
            class1 = class_ids[i]

            # Initialize merged box with the highest confidence detection
            merged_box = [x1, y1, x1 + w1, y1 + h1]  # [x_min, y_min, x_max, y_max]
            indices_to_remove = [i]

            # Look for overlapping boxes to merge
            for j in indices[1:]:
                x2, y2, w2, h2 = boxes[j]
                box2 = [x2, y2, x2 + w2, y2 + h2]

                # Calculate IoU
                x_left = max(merged_box[0], box2[0])
                y_top = max(merged_box[1], box2[1])
                x_right = min(merged_box[2], box2[2])
                y_bottom = min(merged_box[3], box2[3])

                intersection = max(0, x_right - x_left) * max(0, y_bottom - y_top)
                area1 = (merged_box[2] - merged_box[0]) * (merged_box[3] - merged_box[1])
                area2 = (box2[2] - box2[0]) * (box2[3] - box2[1])
                union = area1 + area2 - intersection
                iou = intersection / union if union > 0 else 0

                # Merge if IoU is high or boxes are likely parts of the same person
                if iou > 0.3 or (abs(merged_box[0] - box2[0]) < w1 * 0.5 and abs(merged_box[1] - box2[1]) < h1 * 0.5):
                    # Expand the merged box to include the second box
                    merged_box[0] = min(merged_box[0], box2[0])
                    merged_box[1] = min(merged_box[1], box2[1])
                    merged_box[2] = max(merged_box[2], box2[2])
                    merged_box[3] = max(merged_box[3], box2[3])
                    indices_to_remove.append(j)

            # Convert back to [x, y, w, h] format
            merged_x = merged_box[0]
            merged_y = merged_box[1]
            merged_w = merged_box[2] - merged_box[0]
            merged_h = merged_box[3] - merged_box[1]

            merged_boxes.append([merged_x, merged_y, merged_w, merged_h])
            merged_confidences.append(conf1)
            merged_class_ids.append(class1)

            # Remove merged indices
            indices = [idx for idx in indices if idx not in indices_to_remove]

        return merged_boxes, merged_confidences, merged_class_ids

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

        boxes, confidences, class_ids = self.detect(frame)
        indices = cv2.dnn.NMSBoxes(boxes, confidences, self.conf_threshold, self.nms_threshold)

        if len(indices) > 0:
            indices = indices.flatten()
            # Extract boxes, confidences, and class_ids after NMS
            nms_boxes = [boxes[i] for i in indices]
            nms_confidences = [confidences[i] for i in indices]
            nms_class_ids = [class_ids[i] for i in indices]

            # Merge overlapping boxes to ensure full-body detection
            merged_boxes, merged_confidences, merged_class_ids = self.merge_boxes(nms_boxes, nms_confidences, nms_class_ids)

            # Draw the merged boxes
            for i in range(len(merged_boxes)):
                x, y, w, h = merged_boxes[i]
                label = f"{self.classes[merged_class_ids[i]]} {merged_confidences[i]:.2f}"
                cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 1)
                cv2.putText(frame, label, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

        with self.lock:
            self.display_frame = frame
            self.new_frame_available = True

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

    def display_loop(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.new_frame_available:
                with self.lock:
                    if self.display_frame is None:
                        continue
                    frame = self.display_frame
                    self.new_frame_available = False
                cv2.imshow("YOLOv3-Tiny - Detect P", frame)
            if cv2.waitKey(1) == 27:
                rospy.signal_shutdown("ESC pressed")
            rate.sleep()

if __name__ == '__main__':
    try:
        detector = YOLOPersonDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()