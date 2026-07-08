#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class JetbotMotorNode(Node):
    def __init__(self):
        super().__init__('jetbot_motor_node')
        
        # 1. Đăng ký Subscriber lắng nghe topic /cmd_vel từ Laptop bay sang qua Wi-Fi
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )
        
        # 2. Các thông số hình học xe của ông
        self.wheel_radius = 0.0325      
        self.wheel_separation = 0.135   
        self.max_motor_speed = 10.0
        
        # 3. KHỞI TẠO PHẦN CỨNG ĐỘNG CƠ Ở ĐÂY
        # (Ví dụ: Khởi tạo thư viện PCA9685, Adafruit Motor Kit, hoặc chân GPIO...)
        self.get_logger().info('ROS 2 Jetbot: Đã thông mạng, đang lắng nghe /cmd_vel từ Laptop...')

    def execute_motor_command(self, left_speed, right_speed):
        """
        HÀM NÀY ĐỂ ÔNG ĐIỀN CODE ĐIỀU KHIỂN PHẦN CỨNG THẬT
        left_speed và right_speed có giá trị từ -1.0 đến 1.0
        """
        # [Ông điền bùa chú điều khiển mạch cầu H hoặc I2C vào đây nhe]
        # Ví dụ nháp:
        # self.motor_left.set_speed(left_speed)
        # self.motor_right.set_speed(right_speed)
        
        self.get_logger().info(f'Đang chạy bánh -> Trái: {left_speed:.2f}, Phải: {right_speed:.2f}')

    def cmd_vel_callback(self, msg):
        # Lấy vận tốc tuyến tính và vận tốc góc từ topic /cmd_vel
        v = msg.linear.x * 0.5   
        w = msg.angular.z * 2.0 

        # Toán động học nghịch hệ vi sai bằng LaTeX cho ông dễ nhìn:
        # $v_{left} = v - \frac{w \cdot L}{2}$
        # $v_{right} = v + \frac{w \cdot L}{2}$
        v_left = v - (w * self.wheel_separation / 2.0)
        v_right = v + (w * self.wheel_separation / 2.0)

        w_left = v_left / self.wheel_radius
        w_right = v_right / self.wheel_radius

        # Ép dải tốc độ chuẩn về [-1.0, 1.0]
        left_cmd = max(min(w_left / self.max_motor_speed, 1.0), -1.0)
        right_cmd = max(min(w_right / self.max_motor_speed, 1.0), -1.0)

        # Gọi hàm điều khiển phần cứng thực tế
        self.execute_motor_command(left_cmd, right_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = JetbotMotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Khi ngắt node (Ctrl+C), cho xe dừng khẩn cấp tránh đâm tường
        node.execute_motor_command(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()