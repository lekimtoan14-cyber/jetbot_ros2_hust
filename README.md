# Robot Di Động Dẫn Động Vi Sai Sử Dụng ROS2

Dự án xây dựng robot di động dẫn động vi sai sử dụng **ROS2 Humble**, **Jetson Nano**, **ESP32** và **YDLIDAR X4** nhằm thực hiện bài toán điều khiển, định vị (Odometry), lập bản đồ (SLAM) và điều hướng trong môi trường trong nhà.

---

# Kiến trúc hệ thống

Hệ thống gồm ba khối chính:

- **Jetson Nano**
  - Chạy ROS2 Humble
  - Điều khiển động cơ
  - Tính toán Odometry
  - Thực hiện thuật toán SLAM

- **ESP32**
  - Đọc Encoder hai bánh xe
  - Gửi dữ liệu Encoder về Jetson Nano qua Serial

- **Laptop**
  - Điều khiển Robot từ bàn phím
  - Hiển thị dữ liệu bằng RViz2
  - Thực hiện quá trình Mapping

---

# Phần cứng sử dụng

- NVIDIA Jetson Nano
- ESP32 DevKit V1
- Waveshare JetBot Expansion Board
- YDLIDAR X4
- Động cơ DC Encoder
- Pin 12V

---

# Phần mềm sử dụng

- Ubuntu 20.04
- ROS2 Humble
- Python 3
- Google Cartographer
- Navigation2
- RViz2
- Arduino IDE

---

# Cấu trúc Workspace

```
hri_ws
│
├── src
│   ├── my_bot
│   ├── ydlidar_ros2_driver
│   └── ...
│
├── build
├── install
├── log
└── maps
```

---

# 1. Nạp chương trình cho ESP32

## Kết nối Encoder

| Encoder | ESP32 |
|----------|--------|
| Trái A | GPIO18 |
| Trái B | GPIO19 |
| Phải A | GPIO22 |
| Phải B | GPIO23 |

---

## Nạp chương trình

Mở **Arduino IDE**

```
File
    ↓
Open
    ↓
esp32_encoder.ino
```

Chọn

```
Board:
DOIT ESP32 DEVKIT V1
```

Chọn đúng cổng COM.

Nhấn **Upload**.

Nếu Arduino IDE dừng ở

```
Connecting......
```

thì giữ nút **BOOT** trên ESP32 cho đến khi quá trình nạp bắt đầu.

Sau khi nạp xong, mở **Serial Monitor** với tốc độ

```
115200 baud
```

Dữ liệu trả về sẽ có dạng

```
LeftTicks,RightTicks
```

---

# 2. Kết nối Jetson Nano

Cắm cáp Micro-USB từ Jetson Nano vào Laptop.

SSH vào Jetson

```bash
ssh ubuntu@192.168.55.1
```

Kết nối Wi-Fi

```bash
sudo nmcli dev wifi connect "Tên_Wifi" password "Mật_khẩu"
```

Sau khi kết nối thành công, tìm địa chỉ IP mới

```bash
sudo arp-scan --localnet
```

Đăng nhập lại

```bash
ssh jetson@ĐỊA_CHỈ_IP
```

Mật khẩu

```
jetson
```

---

# 3. Cài đặt Driver LiDAR

```bash
cd ~/hri_ws/src

git clone https://github.com/lekimtoan14-cyber/ydlidar_ros2_driver.git

cd ~/hri_ws

colcon build --packages-select ydlidar_ros2_driver

source install/setup.bash
```

---

# 4. Khởi động Driver LiDAR

```bash
ros2 launch ydlidar_ros2_driver ydlidar_launch.py
```

---

# 5. Khởi động Node điều khiển động cơ

```bash
cd ~/hri_ws

sudo chmod 777 /dev/ttyUSB1

python3 motor_node.py
```

Node sẽ:

- Nhận lệnh vận tốc (`/cmd_vel`)
- Điều khiển động cơ
- Tính Odometry
- Publish TF
- Publish topic `/odom`

---

# 6. Khởi động mô hình Robot

Terminal thứ nhất

```bash
ros2 run robot_state_publisher robot_state_publisher \
--ros-args \
-p robot_description:="$(xacro ~/hri_ws/src/my_bot/description/robot.urdf.xacro)"
```

Terminal thứ hai

```bash
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```

---

# 7. Khởi động Google Cartographer

Terminal thứ ba

```bash
ros2 run cartographer_ros cartographer_node \
-configuration_directory ~ \
-configuration_basename my_robot.lua \
--ros-args \
--remap scan:=/scan
```

Terminal thứ tư

```bash
ros2 run cartographer_ros \
cartographer_occupancy_grid_node \
-resolution 0.05 \
-publish_period_sec 1.0
```

---

# 8. Điều khiển Robot bằng bàn phím

Cho phép Docker sử dụng giao diện X11

```bash
xhost +local:root
```

Khởi động Docker

```bash
sudo docker run -it \
--net=host \
--privileged \
-e DISPLAY=$DISPLAY \
-v /tmp/.X11-unix:/tmp/.X11-unix \
-v /home/ton_/hri_ws:/root/hri_ws \
osrf/ros:foxy-desktop
```

Trong Docker

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

# 9. Hiển thị trên RViz2

Khởi động RViz2

```bash
rviz2
```

Thêm các Display

- RobotModel
- TF
- LaserScan
- Map

---

# 10. Quy trình lập bản đồ

Sau khi toàn bộ hệ thống được khởi động:

- Điều khiển Robot di chuyển với tốc độ chậm.
- Hạn chế quay nhanh hoặc đổi hướng đột ngột.
- Di chuyển theo quỹ đạo khép kín để thuật toán Loop Closure hoạt động hiệu quả.
- Quan sát quá trình tạo bản đồ trên RViz2.

---

# 11. Lưu bản đồ

```bash
cd ~/hri_ws

ros2 run nav2_map_server map_saver_cli -f my_hust_map
```

Sau khi hoàn thành sẽ tạo hai tệp:

```
my_hust_map.pgm

my_hust_map.yaml
```

---

# Kết quả đạt được

Hệ thống Robot có khả năng:

- Điều khiển động cơ bằng ROS2.
- Đọc Encoder và tính toán Odometry.
- Thu thập dữ liệu từ YDLIDAR X4.
- Xây dựng bản đồ bằng Google Cartographer.
- Hiển thị Robot trên RViz2.
- Lưu bản đồ phục vụ Navigation.

Việc kết hợp dữ liệu Encoder và LiDAR giúp cải thiện độ chính xác của quá trình định vị, giảm sai số tích lũy và nâng cao chất lượng bản đồ trong quá trình robot di chuyển.

---

# Tác giả

**Lê Kim Toàn**

Trường Đại học Bách khoa Hà Nội (HUST)

Ngành Cơ điện tử
