#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys, termios, tty

LINEAR_SPEED = 0.5  
ANGULAR_SPEED = 1.0  

key_mapping = {
    'w': (LINEAR_SPEED, 0),  
    's': (-LINEAR_SPEED, 0),  
    'a': (0, ANGULAR_SPEED),  
    'd': (0, -ANGULAR_SPEED), 
    'q': "exit"               
}

def get_key():
    """Hàm đọc phím nhấn từ bàn phím mà không cần nhấn Enter."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)  
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

def teleop_control():
    """Hàm chính để điều khiển xe bằng phím WASD."""
    rospy.init_node("teleop_car", anonymous=True)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    rospy.loginfo("=== TELEOP CONTROL (WASD) ===")
    rospy.loginfo("W - Tiến | S - Lùi | A - Trái | D - Phải | Q - Thoát")

    twist = Twist()
    rate = rospy.Rate(10)  # Tốc độ lặp 10Hz

    while not rospy.is_shutdown():
        key = get_key()

        if key in key_mapping:
            if key_mapping[key] == "exit":
                rospy.loginfo("Thoát chương trình!")
                break

            linear, angular = key_mapping[key]
            twist.linear.x = linear
            twist.angular.z = angular

            pub.publish(twist)
            rospy.loginfo(f"Linear: {twist.linear.x}, Angular: {twist.angular.z}")

        elif key == '\x03': 
            break
        
        rate.sleep()

if __name__ == "__main__":
    try:
        teleop_control()
    except rospy.ROSInterruptException:
        pass
