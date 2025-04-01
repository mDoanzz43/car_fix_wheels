#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
import geopy.distance

class MoveWithGPS:
    def __init__(self, distance_goal, speed=0.5):
        rospy.init_node('move_with_gps', anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/gps/fix', NavSatFix, self.gps_callback)

        self.start_latlon = None
        self.current_latlon = None

        self.distance_goal = abs(distance_goal)  
        self.speed = speed if distance_goal > 0 else -speed  
        self.moving = False

    def gps_callback(self, msg):
        """Hàm callback để cập nhật vị trí từ GPS."""
        self.current_latlon = (msg.latitude, msg.longitude)

        if self.start_latlon is None:
            self.start_latlon = self.current_latlon
            rospy.loginfo(f"📍 Điểm bắt đầu: {self.start_latlon}")

        if self.moving and self.start_latlon is not None:
            distance_traveled = geopy.distance.distance(self.start_latlon, self.current_latlon).m
            rospy.loginfo(f"🚗 Đã đi: {distance_traveled:.2f}m / {self.distance_goal}m")

            if distance_traveled >= self.distance_goal - 0.05:
                self.stop()
                rospy.loginfo("✅ Xe đã đi đủ khoảng cách!")

    def move(self):
        """Bắt đầu di chuyển."""
        rospy.loginfo(f"🚀 Xe bắt đầu di chuyển {self.distance_goal}m...")
        self.moving = True
        twist = Twist()
        twist.linear.x = self.speed  
        self.cmd_vel_pub.publish(twist)

    def stop(self):
        """Dừng xe khi đạt khoảng cách mong muốn."""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)  
        self.moving = False
        rospy.signal_shutdown("🏁 Hoàn thành di chuyển.")

if __name__ == "__main__":
    try:
        x = int(input("Nhập số mét muốn di chuyển (dương: đi thẳng, âm: đi lùi): "))
        mover = MoveWithGPS(distance_goal=x)
        rospy.sleep(2)  
        mover.move()
        rospy.spin()
    except ValueError:
        rospy.logerr("Vui lòng nhập một số nguyên hợp lệ!")
    except rospy.ROSInterruptException:
        pass
