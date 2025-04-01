#!/usr/bin/env python3

import rospy
import sys, termios, tty, select
from geometry_msgs.msg import Twist

# Thông số xe
WHEEL_RADIUS = 0.35  # Bán kính bánh xe (m)
WHEEL_BASE = 0.45     # Khoảng cách giữa hai bánh xe (m)
LINEAR_SPEED = 0.5   # Vận tốc tuyến tính (m/s)
ANGULAR_SPEED = 1.0  # Vận tốc góc (rad/s)

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

def compute_wheel_velocities(linear_x, angular_z):
    """Tính toán vận tốc bánh xe trái và phải từ vận tốc xe."""
    v_l = (linear_x - (angular_z * WHEEL_BASE / 2)) / WHEEL_RADIUS
    v_r = (linear_x + (angular_z * WHEEL_BASE / 2)) / WHEEL_RADIUS
    return v_l, v_r

def teleop_control():
    rospy.init_node("teleop_diff_drive", anonymous=True)
    cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    
    rospy.loginfo("=== TELEOP CONTROL ===")
    rospy.loginfo("W - Tiến | S - Lùi | A - Trái | D - Phải | X - Dừng | Q - Thoát")
    
    twist = Twist()
    rate = rospy.Rate(10)  # 10Hz

    while not rospy.is_shutdown():
        key = get_key()
        
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
        rospy.loginfo(f"Vận tốc: x={twist.linear.x:.2f} m/s, z={twist.angular.z:.2f} rad/s")
        rate.sleep()

if __name__ == "__main__":
    try:
        teleop_control()
    except rospy.ROSInterruptException:
        pass
