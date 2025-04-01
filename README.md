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
![image](https://github.com/user-attachments/assets/d6217b9e-e087-4af9-95e6-9f6c413db5a2)
