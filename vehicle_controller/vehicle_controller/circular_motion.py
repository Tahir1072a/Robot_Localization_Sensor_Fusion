#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import math

class CircularMotionController(Node):
    def __init__(self):
        super().__init__('circular_motion_controller')
    
        self.declare_parameter('linear_speed', 1.0)  # m/s
        self.declare_parameter('radius', 1.0)        # metre
        self.declare_parameter('clockwise', True)    # Saat yönü kontrolü
        
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/diff_drive_robot_controller/cmd_vel', 10)
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # Parametreleri al
        self.linear_speed = self.get_parameter('linear_speed').value
        self.radius = self.get_parameter('radius').value
        self.clockwise = self.get_parameter('clockwise').value
        
        # Angular hız => (v = ωr)
        self.angular_speed = self.linear_speed / self.radius

        self.get_logger().info(
            f'Dairesel hareket başlatılıyor:\n'
            f'Yarıçap: {self.radius}m\n'
            f'Doğrusal Hız: {self.linear_speed}m/s\n'
            f'Açısal Hız: {self.angular_speed}rad/s\n'
            f'Yön: {"Saat yönünde" if self.clockwise else "Saat yönünün tersine"}'
        )

    def timer_callback(self):
        msg = TwistStamped()
    
        # Doğrusal hız
        msg.twist.linear.x = self.linear_speed
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0
        
        # Açısal hız (saat yönü için negatif)
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = -self.angular_speed if self.clockwise else self.angular_speed
        
        self.cmd_vel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    
    controller = CircularMotionController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        stop_msg = TwistStamped()
        """
        stop_msg.header.stamp = controller.get_clock().now().to_msg()
        stop_msg.header.frame_id = "base_link"
        """
        controller.cmd_vel_pub.publish(stop_msg)
        controller.get_logger().info('Dairesel hareket durduruldu')
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()