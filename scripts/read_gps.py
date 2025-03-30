#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix

def gps_callback(msg):
    """Hàm callback để hiển thị toàn bộ dữ liệu từ topic /gps/fix"""
    gps_data = f"""
-------------------------------
📍 GPS Data:
🔹 Header:
   - Seq: {msg.header.seq}
   - Stamp: {msg.header.stamp.secs}.{msg.header.stamp.nsecs}
   - Frame ID: {msg.header.frame_id}
🔹 Status:
   - Status: {msg.status.status}
   - Service: {msg.status.service}
🔹 Position:
   - Latitude: {msg.latitude}
   - Longitude: {msg.longitude}
   - Altitude: {msg.altitude}
🔹 Covariance:
   - {msg.position_covariance}
   - Covariance Type: {msg.position_covariance_type}
-------------------------------
    """
    rospy.loginfo(gps_data)

def gps_listener():
    """Khởi tạo node và subscriber"""
    rospy.init_node('gps_listener', anonymous=True)
    rospy.Subscriber("/gps/fix", NavSatFix, gps_callback)
    
    rospy.loginfo("🔄 Đang lắng nghe dữ liệu GPS trên /gps/fix...")
    rospy.spin()  # Giữ chương trình chạy liên tục

if __name__ == "__main__":
    try:
        gps_listener()
    except rospy.ROSInterruptException:
        pass
