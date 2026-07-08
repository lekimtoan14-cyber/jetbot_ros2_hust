#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ĐỀ TÀI: NGHIÊN CỨU CHẾ TẠO ROBOT DI ĐỘNG TỰ HÀNH
MÃ NGUỒN: NODE ĐIỀU KHIỂN ĐỘNG CƠ VÀ TRÍCH XUẤT ĐỊNH VỊ QUÁN TÍNH ODOMETRY
NỀN TẢNG: ROS 2 (CRUISE / FOXY / HUMBLE) TRÊN NVIDIA JETSON NANO
"""

from jetbot import Robot
import rclpy
from rclpy.node import Node
import math
import time
import serial
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros
from simple_pid import PID 

class JetsonIndustrialOdomNode(Node):
    def __init__(self):
        super().__init__('jetson_industrial_odom_node')

        # =====================================================================
        # ⚙️ 1. KHỞI TẠO VÀ LIÊN KẾT PHẦN CỨNG NGOẠI VI
        # =====================================================================
        self.robot = None
        self.ser = None

        # Khởi tạo bo mạch driver công suất động cơ của Waveshare
        try:
            self.robot = Robot() 
        except Exception as e:
            self.get_logger().error(f'LỖI KHỞI TẠO MẠCH CÔNG SUẤT WAVESHARE: {str(e)}')

        # Thiết lập kết nối Serial thu nhận xung thô từ vi điều khiển ESP32
        try:
            self.ser = serial.Serial('/dev/ttyUSB1', 115200, timeout=0.01) 
        except Exception as e:
            self.get_logger().error(f'LỖI KẾT NỐI VẬT LÝ SERIAL (ESP32): {str(e)}')

        # Tham số kích thước hình học thực tế hệ thống xe sau hiệu chuẩn
        self.WHEEL_RADIUS = 0.0315        # Bán kính bánh xe thực tế (31.5 mm)
        self.WHEEL_SEPARATION = 0.140     # Khoảng cách giữa hai tâm bánh xe (140 mm)
        self.TICKS_PER_REV = 1092.0       # Tổng số xung/vòng của Encoder DC Servo SK51D

        # Bộ đệm lưu vết giá trị xung tổng tích lũy của chu kỳ trích mẫu trước
        self.last_total_left = None
        self.last_total_right = None

        # Biến tích phân trạng thái vị trí toàn cục trong không gian phẳng Odometry
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0

        # Khởi tạo bộ đo thời gian hệ thống độ chính xác cao để tính toán dt động
        self.last_time = time.monotonic()

        # Cấu hình bộ điều khiển phản hồi vòng kín PID thông qua thư viện simple-pid
        self.pid_left = PID(0.002, 0.0003, 0.0, setpoint=0.0)
        self.pid_right = PID(0.002, 0.0003, 0.0, setpoint=0.0)
        
        # Giới hạn biên đầu ra (Output Limits) để chống bão hòa tích phân (Anti-Windup)
        self.pid_left.output_limits = (-0.15, 0.15)
        self.pid_right.output_limits = (-0.15, 0.15)

        # Khởi tạo các đại lượng vận tốc góc (RPM) mục tiêu và RPM thực tế
        self.target_rpm_left = 0.0
        self.target_rpm_right = 0.0
        self.real_rpm_left = 0.0
        self.real_rpm_right = 0.0

        # Thiết lập hạ tầng mạng truyền thông điệp của hệ điều hành ROS 2
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Khởi tạo bộ định thì (Timer) kích hoạt vòng điều khiển tần số 50Hz (Chu kỳ 20ms)
        self.log_counter = 0
        self.timer = self.create_timer(0.02, self.control_loop) 
        self.get_logger().info('🚀 ĐÃ KÍCH HOẠT THÀNH CÔNG NODE ĐIỀU KHIỂN ĐỘNG CƠ VÀ TRÍCH XUẤT ODOMETRY!')

    def cmd_vel_callback(self, msg):
        """ Hàm callback tiếp nhận vận tốc hình học từ topic /cmd_vel và giải động học nghịch """
        # Áp đặt ngưỡng chặn trần an toàn động học cho robot di động
        MAX_LINEAR = 0.1   # Tốc độ tịnh tiến tối đa (m/s)
        MAX_ANGULAR = 0.4  # Tốc độ quay tối đa (rad/s)
        
        linear = max(min(msg.linear.x, MAX_LINEAR), -MAX_LINEAR)
        angular = max(min(msg.angular.z, MAX_ANGULAR), -MAX_ANGULAR)

        # Phương trình toán học động học nghịch đảo cho hệ thống dẫn động vi sai
        v_left = linear - (angular * self.WHEEL_SEPARATION * 0.5)
        v_right = linear + (angular * self.WHEEL_SEPARATION * 0.5)

        # Chuyển đổi đơn vị từ vận tốc tuyến tính (m/s) sang vận tốc vòng của trục động cơ (RPM)
        self.target_rpm_left = (v_left * 60.0) / (2.0 * math.pi * self.WHEEL_RADIUS)
        self.target_rpm_right = (v_right * 60.0) / (2.0 * math.pi * self.WHEEL_RADIUS)
        
        # Cập nhật giá trị đặt mục tiêu (Setpoint) cho hai bộ điều khiển PID độc lập
        self.pid_left.setpoint = self.target_rpm_left
        self.pid_right.setpoint = self.target_rpm_right

    def calc_feed_forward_left(self, rpm):
        """ Hàm toán học hồi quy tuyến tính tính toán giá trị điện áp mồi (Feed-Forward) bánh trái """
        if abs(rpm) < 1.0: 
            return 0.0
        return (0.00457 * rpm + 0.0556) if rpm > 0 else (0.00457 * rpm - 0.0556)

    def calc_feed_forward_right(self, rpm):
        """ Hàm toán học hồi quy tuyến tính tính toán giá trị điện áp mồi (Feed-Forward) bánh phải """
        if abs(rpm) < 1.0: 
            return 0.0
        return (0.00519 * rpm + 0.0650) if rpm > 0 else (0.00519 * rpm - 0.0650)

    def control_loop(self):
        """ Vòng lặp điều khiển thời gian thực chu kỳ cố định 20ms """
        # Kiểm tra điều kiện an toàn kết nối ngoại vi cổng Serial
        if self.ser is None or not self.ser.is_open: 
            return

        # ⏱️ Tính toán khoảng thời gian vi phân thực tế dt giữa hai chu kỳ liên tiếp
        now = time.monotonic()
        dt = now - self.last_time
        self.last_time = now

        if dt <= 0.0: 
            dt = 0.001

        accumulated_delta_left = 0
        accumulated_delta_right = 0
        has_data = False

        # 📥 Đọc và bóc tách dữ liệu chuỗi xung thô truyền liên tục từ ESP32
        while self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not line: 
                    continue
                data = line.split(',')
                if len(data) == 2:
                    current_left = int(data[0])
                    current_right = int(data[1])

                    # Xác lập mốc lấy mẫu xung cơ sở tại thời điểm khởi động node
                    if self.last_total_left is None:
                        self.last_total_left = current_left
                    if self.last_total_right is None:
                        self.last_total_right = current_right

                    # Tính toán khoảng dịch sai lệch xung (Delta Ticks) trong chu kỳ lấy mẫu
                    accumulated_delta_left += (current_left - self.last_total_left)
                    accumulated_delta_right += (current_right - self.last_total_right)
                    has_data = True

                    self.last_total_left = current_left
                    self.last_total_right = current_right
            except Exception:
                continue

        # Cơ chế an toàn: Nếu mất gói tin chu kỳ, ép sai lệch xung về 0 để bảo vệ hệ thống
        if not has_data:
            accumulated_delta_left = 0
            accumulated_delta_right = 0

        # Tính toán vận tốc thực tế tức thời (RPM) của hai bánh xe dựa trên số xung thô và dt
        self.real_rpm_left = (accumulated_delta_left * 60.0) / (self.TICKS_PER_REV * dt)
        self.real_rpm_right = (accumulated_delta_right * 60.0) / (self.TICKS_PER_REV * dt)

        # 📐 THUẬT TOÁN ĐỒNG HỌC THUẬN TRÍCH XUẤT QUỸ ĐẠO ODOMETRY CHUẨN ĐẠI HỌC BÁCH KHOA
        d_left = (accumulated_delta_left / self.TICKS_PER_REV) * (2.0 * math.pi * self.WHEEL_RADIUS)
        d_right = (accumulated_delta_right / self.TICKS_PER_REV) * (2.0 * math.pi * self.WHEEL_RADIUS)

        # Tính toán khoảng dịch quãng đường trung bình và độ chênh lệch góc quay
        average_distance = (d_right + d_left) * 0.5
        d_yaw = (d_right - d_left) / self.WHEEL_SEPARATION
        average_angle = self.odom_yaw + d_yaw * 0.5

        # Tích phân Euler cập nhật trạng thái tọa độ vị trí toàn cục của robot (X, Y, Yaw)
        self.odom_x += math.cos(average_angle) * average_distance
        self.odom_y += math.sin(average_angle) * average_distance
        self.odom_yaw += d_yaw

        # Thuật toán chuẩn hóa góc hướng (Orientation Profile) nằm gọn trong đoạn toán học [-PI, +PI]
        self.odom_yaw = (self.odom_yaw + math.pi) % (2.0 * math.pi) - math.pi

        # 🎮 THUẬT TOÁN ĐIỀU KHIỂN PHỐI HỢP VÒNG HỞ FEED-FORWARD VÀ VÒNG KÍN CLOSED-LOOP PID
        if abs(self.target_rpm_left) < 0.1 and abs(self.target_rpm_right) < 0.1:
            final_pwm_left = 0.0
            final_pwm_right = 0.0
            self.pid_left.reset()
            self.pid_right.reset()
        else:
            final_pwm_left = self.calc_feed_forward_left(self.target_rpm_left) + self.pid_left(self.real_rpm_left)
            final_pwm_right = self.calc_feed_forward_right(self.target_rpm_right) + self.pid_right(self.real_rpm_right)

        # Giới hạn ngưỡng trần độ rộng xung PWM tối đa gửi đến mạch công suất Waveshare
        final_pwm_left = max(min(final_pwm_left, 0.85), -0.85)
        final_pwm_right = max(min(final_pwm_right, 0.85), -0.85)

        # Thúc dòng công suất điều khiển quay motor thông qua đối tượng phần cứng Waveshare
        if self.robot is not None:
            self.robot.set_motors(final_pwm_left, -final_pwm_right)

        # =====================================================================
        # 📤 2. ĐÓNG GÓI VÀ XUẤT BẢN THÔNG ĐIỆP NAV_MSGS/ODOMETRY LÊN HỆ THỐNG ROS 2
        # =====================================================================
        current_ros_time = self.get_clock().now()
        odom = Odometry()
        odom.header.stamp = current_ros_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # Cấu hình dữ liệu ma trận vị trí hình học (Pose) và góc Euler chuyển đổi sang Quaternion
        odom.pose.pose.position.x = self.odom_x
        odom.pose.pose.position.y = self.odom_y
        odom.pose.pose.orientation.z = math.sin(self.odom_yaw * 0.5)
        odom.pose.pose.orientation.w = math.cos(self.odom_yaw * 0.5)

        # Thiết lập ma trận hiệp phương sai sai số vị trí (Covariance Matrix) cho bộ lọc Kalman tầng trên
        odom.pose.covariance = [
            0.01,  0.0,  0.0,     0.0,     0.0,     0.0,
            0.0,   0.01, 0.0,     0.0,     0.0,     0.0,
            0.0,   0.0,  99999.0, 0.0,     0.0,     0.0,
            0.0,   0.0,  0.0,     99999.0, 0.0,     0.0,
            0.0,   0.0,  0.0,     0.0,     99999.0, 0.0,
            0.0,   0.0,  0.0,     0.0,     0.0,     0.05
        ]

        # Đóng gói giá trị vận tốc tuyến tính và vận tốc góc thực tế hiện tại của robot
        odom.twist.twist.linear.x = average_distance / dt
        odom.twist.twist.angular.z = d_yaw / dt

        odom.twist.covariance = [
            0.02,  0.0,  0.0,     0.0,     0.0,     0.0,
            0.0,   0.02, 0.0,     0.0,     0.0,     0.0,
            0.0,   0.0,  99999.0, 0.0,     0.0,     0.0,
            0.0,   0.0,  0.0,     99999.0, 0.0,     0.0,
            0.0,   0.0,  0.0,     0.0,     99999.0, 0.0,
            0.0,   0.0,  0.0,     0.0,     0.0,     0.1
        ]

        # Thực hiện xuất bản dữ liệu định vị quán tính lên hệ thống ROS 2 topic
        self.odom_pub.publish(odom)

        # =====================================================================
        # 🔗 3. PHÁT QUẢNG BÁ PHÉP BIẾN ĐỔI HỆ TỌA ĐỘ VÒNG TRONG (TF TREE: ODOM -> BASE_LINK)
        # =====================================================================
        t = TransformStamped()
        t.header.stamp = current_ros_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        t.transform.translation.x = self.odom_x
        t.transform.translation.y = self.odom_y
        t.transform.rotation.z = math.sin(self.odom_yaw * 0.5)
        t.transform.rotation.w = math.cos(self.odom_yaw * 0.5)
        
        self.tf_broadcaster.sendTransform(t)

        # Cơ chế ghi log tuần tự giám sát chu kỳ đáp ứng động thực tế hệ thống (1 giây/lần)
        self.log_counter += 1
        if self.log_counter % 50 == 0:
            self.get_logger().info(
                f"📊 [CONTROL] Target RPM: [L:{self.target_rpm_left:.1f}, R:{self.target_rpm_right:.1f}] | "
                f"Real RPM: [L:{self.real_rpm_left:.1f}, R:{self.real_rpm_right:.1f}] | "
                f"Pose: [X:{self.odom_x:.2f}, Y:{self.odom_y:.2f}, Yaw:{math.degrees(self.odom_yaw):.1f}°]"
            )
            self.log_counter = 0

def main(args=None):
    rclpy.init(args=args)
    node = JetsonIndustrialOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cơ chế an toàn ngắt cưỡng bức dòng điện cấp cho động cơ khi tắt hệ thống node
        if hasattr(node, 'robot') and node.robot is not None:
            try:
                node.robot.set_motors(0.0, 0.0)
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()