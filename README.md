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
```catkin_make``` -> ```source devel/setup.bash```
### Các bước thực hiện
#### 1. Chạy Gazebo và Rviz
```roslaunch car_fix_wheels display.launch```
##### Hình ảnh trong mô phỏng
![image](https://github.com/user-attachments/assets/cafd41bc-ec39-40a1-b222-8e21ccae0555)
#### 2. Cấp quyền cho các file python
```chmod +x control_arm_car.py control_arm.py control_car_without_lib.py control_car.py read_gps.py run_x_meters_gps.py```
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
##### Kết quả
![image](https://github.com/user-attachments/assets/4dbdaf52-18b8-4e3d-9577-abe4ebd02afb)
