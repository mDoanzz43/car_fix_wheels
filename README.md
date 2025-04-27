# Thiết kế và triển khai Robot 2 bánh vi sai tích hợp Slam, Navigation, Object Detection trong môi trường Gazebo và Rviz
## Description:
Xây dựng và điều khiển xe 2 bánh vi sai. Xe có thể sử dụng keyboard để điều kiển bánh xe và tay máy; hiển thị các dữ liệu cảm biến: Lidar, Camera, GPS, Imu. Sử dụng thuật toán Gmapping cho SLAM; Navigation; Sử dụng mô hình Yolov3 cho bài toán Object Detection để phát hiện được người trong môi trường Gazebo và Rviz.
### 1. Setup môi trường
- ROS, Gazebo, Rviz
- Download source:
```git clone https://github.com/mDoanzz43/Ros_project.git```
- Cài đặt môi trường cho python:
```pip install -r requirements.txt```
- Cài đặt thư viện để đọc GPS:
```sudo apt-get install ros-noetic-hector-gazebo-plugins```
- Đảm bảo không gian làm việc:
```catkin_make``` -> ```source devel/setup.bash```
- Export path: export GAZEBO_MODEL_PATH=~/ck2_ws/src/car_fix_wheels/models:$GAZEBO_MODEL_PATH
### Các bước thực hiện
#### 1. Chạy Gazebo và Rviz
```roslaunch car_fix_wheels display.launch```
##### Hình ảnh trong mô phỏng
![image](https://github.com/user-attachments/assets/cafd41bc-ec39-40a1-b222-8e21ccae0555)
#### 2. Cấp quyền cho các file python
```chmod +x control_arm_car.py control_arm.py control_car_without_lib.py control_car.py read_gps.py run_x_meters_gps.py detect.py```
#### 3. Điều khiển xe
```rosrun car_fix_wheels control_car.py``` 
##### 3.1. Điều khiển xe dựa trên động lực học
```rosrun car_fix_wheels control_car_without_lib.py```
#### 4. Điều khiển tay máy
```rosrun car_fix_wheels control_arm.py```
#### 5. Điều khiển xe và tay máy 
```rosrun car_fix_wheels control_arm_car.py```
#### 6. Đọc cảm biến GPS
```rosrun car_fix_wheels read_gps.py```
#### 7. Di chuyển quãng đường chính xác sử dụng cảm biến GPS
```rosrun run_x_meters_gps.py```
#### 8. Chạy Slam cho Robot 
```roslaunch car_fix_wheels gmapping.launch```
và sử dụng bàn phím để quét map
```rosrun car_fix_wheels control_car_without_lib.py```
#### 7. Chạy Navigation cho Robot
```roslaunch car_fix_wheels gazebo.launch```
```roslaunch car_fix_wheels robot_navigation.launch map_file:=$HOME/map.yaml```
#### 7. Phát hiện ngườ
```roslaunch car_fix_wheels detect.launch```
```rosrun car_fix_wheels detect.py```
```rosrun car_fix_wheels control_car_without_lib.py```
##### Kết quả
![image](https://github.com/user-attachments/assets/4dbdaf52-18b8-4e3d-9577-abe4ebd02afb)
##### Link video demo: 
https://youtu.be/oKi8eqXXKbI
