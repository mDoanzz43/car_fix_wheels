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
  
!Note: Thay thế ck2_ws bằng đường dẫn của bạn
### Các bước thực hiện
#### 1. Chạy Gazebo và Rviz
``` bash 
roslaunch car_fix_wheels display.launch
```

##### Hình ảnh trong mô phỏng
![image](https://github.com/user-attachments/assets/cafd41bc-ec39-40a1-b222-8e21ccae0555)
#### 2. Cấp quyền cho các file python
```bash
chmod +x control_arm_car.py control_arm.py control_car_without_lib.py control_car.py detect.py
```
#### 3. Điều khiển xe
```bash 
rosrun car_fix_wheels control_car.py
``` 
##### 3.1. Điều khiển xe dựa trên động lực học
```bash 
rosrun car_fix_wheels control_car_without_lib.py
```
#### 4. Điều khiển tay máy
```bash 
rosrun car_fix_wheels control_arm.py
```
#### 5. Điều khiển xe và tay máy 
```bash 
rosrun car_fix_wheels control_arm_car.py

#### 6. Chạy Slam cho Robot 
```bash
roslaunch car_fix_wheels gmapping.launch
```
và sử dụng bàn phím để quét map
```bash
rosrun car_fix_wheels control_car_without_lib.py
```
#### 7. Chạy Navigation cho Robot
```bash
roslaunch car_fix_wheels gazebo.launch
```
```bash
roslaunch car_fix_wheels robot_navigation.launch map_file:=$HOME/map.yaml
```
#### 8. Phát hiện người 
! Note: Thay thế 2 path trong file detect.py thành đường dẫn của mình
```bash
roslaunch car_fix_wheels detect.launch
```
```bash 
rosrun car_fix_wheels detect.py
```
```bash
rosrun car_fix_wheels control_car_without_lib.py
```
##### Kết quả
- Robot khi Slam:
  ![image](https://github.com/user-attachments/assets/e27b7a09-62f4-4d05-ba21-1ba754fe08a6)

- Robot khi navigation:
  ![image](https://github.com/user-attachments/assets/7b93ee8c-4b56-435a-8616-57cb954ce15f)

- Robot khi sử dụng model Yolov3_tiny để detect người:
  ![image](https://github.com/user-attachments/assets/8e00bd0d-732f-4bfa-b3b0-44e918b2c6c8)


##### Link video: https://drive.google.com/drive/folders/1CTKG4Qoo8zRXEgRNgq9gwJ19Y8CFDoU-?usp=sharing
