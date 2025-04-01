# Mô phỏng xe 2 bánh vi sai và tay máy gazebo, rviz
## Mô tả
Mô phỏng xe 2 bánh vi sai và tay máy trong gazebo và mô tả trong rviz. Xe có thể di chuyển bằng bàn phím (bao gồm cả xe và tay máy), đồng thời có thể đọc được các cảm biến GPS, Lidar, Camera. 
### 1. Setup môi trường
- ROS, Gazebo, Rviz
- Download source:
```git clone https://github.com/Dat1908/car_fix_wheels.git```
- Cài đặt môi trường cho python:
```pip install -r requirements.txt```
- Cài đặt thư viện để đọc GPS:
```sudo apt-get install ros-noetic-hector-gazebo-plugins```
- Đảm bảo không gian làm việc:
```source devel/setup.bash```
### Các bước thực hiện
#### 1. Chạy Gazebo và Rviz
```roslaunch car_fix_wheels display.launch```
##### Hình ảnh trong mô phỏng
![image](https://github.com/user-attachments/assets/cafd41bc-ec39-40a1-b222-8e21ccae0555)
#### 2. Cấp quyền cho các file python
```chmod +x control_arm_car.py```
```chmod +x control_arm_car.py```
