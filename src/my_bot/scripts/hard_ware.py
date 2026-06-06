#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import socket  # Thư viện mạng mặc định của Python

class LaptopToJetbotBridge(Node):
    def __init__(self):
        super().__init__('hardware_node')
        
        # 1. Đăng ký Subscriber lắng nghe topic bàn phím ngay trên Laptop
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )
        
        # 2. CẤU HÌNH ĐỊA CHỈ IP CỦA JETBOT
        # ===> ÔNG ĐIỀN CHÍNH XÁC ĐỊA CHỈ IP CỦA XE VÀO ĐÂY <===
        self.JETBOT_IP = "192.168.1.121"  
        self.UDP_PORT = 5005
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # 3. Các thông số hình học xe để làm toán động học nghịch
        self.wheel_radius = 0.0325      
        self.wheel_separation = 0.135   
        self.max_motor_speed = 10.0
        
        self.get_logger().info(f'HRI_WS: Sẵn sàng bắn lệnh mạng sang JetBot tại IP: {self.JETBOT_IP}')

    def cmd_vel_callback(self, msg):
        v = msg.linear.x * 0.5   # Vận tốc tiến lùi (m/s)
        w = msg.angular.z * 2 # Vận tốc xoay tròn (rad/s)

        # Làm toán động học nghịch vi sai ngay trên Laptop cho nhẹ não xe
        v_left = v - (w * self.wheel_separation / 2.0)
        v_right = v + (w * self.wheel_separation / 2.0)

        w_left = v_left / self.wheel_radius
        w_right = v_right / self.wheel_radius

        left_cmd = max(min(w_left / self.max_motor_speed, 1.0), -1.0)
        right_cmd = max(min(w_right / self.max_motor_speed, 1.0), -1.0)

        # Gói 2 con số tốc độ thành chuỗi chữ "trái,phải" rồi ném xuyên không khí qua Wi-Fi
        message = f"{left_cmd},{right_cmd}"
        self.sock.sendto(message.encode('utf-8'), (self.JETBOT_IP, self.UDP_PORT))
        
        self.get_logger().info(f'Bắn lệnh UDP -> Trái: {left_cmd:.2f}, Phải: {right_cmd:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = LaptopToJetbotBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Nếu ông tắt node trên Laptop (Ctrl+C), tự động bắn gói "0,0" sang từ xa để xe dừng ngay lập tức
        node.sock.sendto("0.0,0.0".encode('utf-8'), (node.JETBOT_IP, node.UDP_PORT))
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()