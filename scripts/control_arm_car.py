#!/usr/bin/env python3

import rospy
import sys, termios, tty, select
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

LINEAR_SPEED = 0.5  
ANGULAR_SPEED = 1.0  

STEP_SIZE = 0.1
JOINT1_MIN, JOINT1_MAX = -3.14, 3.14
JOINT2_MIN, JOINT2_MAX = -1.57, 1.57

joint_1_angle = 0.0
joint_2_angle = 0.0
def print_instructions():
    """Hiển thị hướng dẫn điều khiển lên màn hình."""
    # os.system("clear")  
    print("=== TELEOP CONTROL ===")
    print("Xe: W - Tiến | S - Lùi | A - Trái | D - Phải | X - Dừng")
    print("Tay máy: I - Joint1+ | K - Joint1- | J - Joint2+ | L - Joint2-")
    print("Nhấn Q để thoát.")
    print("-----------------------------------")
    
def get_key():
    """Hàm đọc phím nhấn từ bàn phím mà không cần nhấn Enter."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

def teleop_control():
    global joint_1_angle, joint_2_angle

    rospy.init_node("teleop_car_arm", anonymous=True)
    cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    joint1_pub = rospy.Publisher("/arm_1_joint_controller/command", Float64, queue_size=10)
    joint2_pub = rospy.Publisher("/arm_2_joint_controller/command", Float64, queue_size=10)
    
    rospy.loginfo("=== TELEOP CONTROL ===")
    rospy.loginfo("Xe: W - Tiến | S - Lùi | A - Trái | D - Phải | X - Dừng")
    rospy.loginfo("Tay máy: I - Joint1+ | K - Joint1- | J - Joint2+ | L - Joint2-")
    rospy.loginfo("Nhấn Q để thoát.")
    
    twist = Twist()
    rate = rospy.Rate(10)  # 10Hz

    while not rospy.is_shutdown():
        key = get_key()
        print_instructions()
        # Điều khiển xe
        if key == 'w':
            twist.linear.x = LINEAR_SPEED
            twist.angular.z = 0.0
        elif key == 's':
            twist.linear.x = -LINEAR_SPEED
            twist.angular.z = 0.0
        elif key == 'a':
            twist.linear.x = 0.0
            twist.angular.z = ANGULAR_SPEED
        elif key == 'd':
            twist.linear.x = 0.0
            twist.angular.z = -ANGULAR_SPEED
        elif key == 'x':
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        elif key == 'q':
            rospy.loginfo("Thoát chương trình!")
            break

        cmd_vel_pub.publish(twist)

        # Điều khiển tay máy
        if key == 'i':
            joint_1_angle = min(joint_1_angle + STEP_SIZE, JOINT1_MAX)
        elif key == 'k':
            joint_1_angle = max(joint_1_angle - STEP_SIZE, JOINT1_MIN)
        elif key == 'j':
            joint_2_angle = min(joint_2_angle + STEP_SIZE, JOINT2_MAX)
        elif key == 'l':
            joint_2_angle = max(joint_2_angle - STEP_SIZE, JOINT2_MIN)
        
        joint1_pub.publish(joint_1_angle)
        joint2_pub.publish(joint_2_angle)
        rospy.loginfo(f"Joint1: {joint_1_angle:.2f}, Joint2: {joint_2_angle:.2f}")
        
        rate.sleep()

if __name__ == "__main__":
    try:
        teleop_control()
    except rospy.ROSInterruptException:
        pass
