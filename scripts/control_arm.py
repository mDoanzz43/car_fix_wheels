#!/usr/bin/env python3

import rospy
import sys, select, termios, tty
from std_msgs.msg import Float64

# Thông số giới hạn góc (radian)
JOINT_1_MIN = -3.14  # -180 độ
JOINT_1_MAX = 3.14   # 180 độ
JOINT_2_MIN = -1.57  # -90 độ
JOINT_2_MAX = 1.57   # 90 độ

# Thông báo điều khiển
msg = """
Điều khiển tay máy:
- Nhấn 'w' để tăng joint_1
- Nhấn 's' để giảm joint_1
- Nhấn 'a' để tăng joint_2
- Nhấn 'd' để giảm joint_2
- Nhấn 'q' để thoát
Giới hạn:
- Joint 1: từ -3.14 đến 3.14 rad
- Joint 2: từ -1.57 đến 1.57 rad
"""

step_size = 0.1  # Bước tăng/giảm góc (radian)

joint_1_angle = 0.0  # Góc theta1
joint_2_angle = 0.0  # Góc theta2

def get_key():
    """Đọc phím từ terminal"""
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def clamp(value, min_value, max_value):
    """Giới hạn giá trị trong khoảng [min_value, max_value]"""
    return max(min_value, min(value, max_value))

def teleop_control():
    global joint_1_angle, joint_2_angle

    # Khởi tạo node ROS
    rospy.init_node('arm_teleop')
    joint1_pub = rospy.Publisher('/arm_1_joint_controller/command', Float64, queue_size=10)
    joint2_pub = rospy.Publisher('/arm_2_joint_controller/command', Float64, queue_size=10)
    rospy.sleep(1)

    print(msg)

    while not rospy.is_shutdown():
        key = get_key()

        # Điều khiển các khớp
        if key == 'w':
            joint_1_angle += step_size
        elif key == 's':
            joint_1_angle -= step_size
        elif key == 'a':
            joint_2_angle += step_size
        elif key == 'd':
            joint_2_angle -= step_size
        elif key == 'q':
            print("\nThoát teleop!")
            break

        # Giới hạn góc của joint_1 và joint_2
        joint_1_angle = clamp(joint_1_angle, JOINT_1_MIN, JOINT_1_MAX)
        joint_2_angle = clamp(joint_2_angle, JOINT_2_MIN, JOINT_2_MAX)

        # Kiểm tra và thông báo nếu đạt giới hạn
        if joint_1_angle == JOINT_1_MIN or joint_1_angle == JOINT_1_MAX:
            rospy.logwarn(f"Joint 1 reached limit: {joint_1_angle:.3f} rad")
        if joint_2_angle == JOINT_2_MIN or joint_2_angle == JOINT_2_MAX:
            rospy.logwarn(f"Joint 2 reached limit: {joint_2_angle:.3f} rad")

        # Gửi lệnh đến các khớp
        rospy.loginfo(f"Joint 1: {joint_1_angle:.3f} rad, Joint 2: {joint_2_angle:.3f} rad")
        joint1_pub.publish(joint_1_angle)
        joint2_pub.publish(joint_2_angle)

    rospy.signal_shutdown("Tắt teleop")

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    try:
        teleop_control()
    except rospy.ROSInterruptException:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)