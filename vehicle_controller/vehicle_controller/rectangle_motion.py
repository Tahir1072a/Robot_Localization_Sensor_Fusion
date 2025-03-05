#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node

from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import TwistStamped

from tf2_msgs.msg import TFMessage
from std_msgs.msg import Bool

class RectangleMotion(Node):
    def __init__(self):
        super().__init__("rectangle_motion")

        self.diff_drive_control_pub = self.create_publisher(TwistStamped, "/diff_drive_robot_controller/cmd_vel", 10)

        self.vehicle_pose_sub = self.create_subscription(TFMessage, "/vehicle/real_pose", self.vehicle_pose_callback, 10)
        self.loop_pub = self.create_publisher(Bool, "loop_info", 10)
        self.frequency = 0.05
        self.timer = self.create_timer(self.frequency, self.loop_info_callback)
        self.is_loop_finished = False

        self.linear_speed = 1.5
        self.angular_speed = 0.35
        self.rectangle_size = 5.5
        self.current_distance_x = 0.0
        self.start_position = None
        self.is_turn = False
        self.is_moving_forward = True
        self.loop_counter = 0

        self.current_yaw = 0.0
        self.target_yaw = 3.14 / 2

    def vehicle_pose_callback(self, msg):
        vehicle_pose = msg.transforms[0]
        if self.start_position is None:
            self.start_position = vehicle_pose.transform.translation
            self.get_logger().info(f"Başlangıç Noktası: {self.start_position}")
            self.loop_counter += 1
            if self.loop_counter > 4:
                self.is_loop_finished = True
                time.sleep(1)
            return
        if self.is_loop_finished:
            return
        current_position = vehicle_pose.transform.translation
        self.current_distance_x = abs(current_position.x - self.start_position.x)
        self.current_distance_y = abs(current_position.y - self.start_position.y)
        orientation_list = [vehicle_pose.transform.rotation.x, vehicle_pose.transform.rotation.y, vehicle_pose.transform.rotation.z, vehicle_pose.transform.rotation.w]
        _, _, self.current_yaw = euler_from_quaternion(orientation_list)

        if self.is_moving_forward and abs(self.current_distance_x - self.rectangle_size) < 0.01 and self.loop_counter % 2 != 0:
            self.stop()
            self.is_moving_forward = False
            self.is_turn = True
        elif self.is_moving_forward and abs(self.current_distance_y - self.rectangle_size) < 0.01 and self.loop_counter % 2 == 0:
            self.stop()
            self.is_moving_forward = False
            self.is_turn = True
        elif self.is_turn and abs(self.current_yaw - self.target_yaw) < 0.005:
            self.stop()
            self.is_turn = False
            
            self.target_yaw = (self.current_yaw + 3.14 / 2)
            if self.target_yaw > 3.14:
                self.target_yaw = 3.14 - self.target_yaw
            self.get_logger().info(f"Yeni Hedef Yaw: {self.target_yaw}")
            self.get_logger().info(f"Araç dönüsü: {self.current_yaw}")
            self.start_position = None
            self.is_moving_forward = True
        elif self.is_moving_forward:
            self.move_forward()
        elif self.is_turn:
            self.turn_right()

    def move_forward(self):
        msg = TwistStamped()
        msg.twist.linear.x = self.linear_speed
        msg.twist.angular.z = 0.0
        self.diff_drive_control_pub.publish(msg)
    
    def turn_right(self):
        msg = TwistStamped()
        msg.twist.linear.x = 0.0
        msg.twist.angular.z = self.angular_speed
        self.diff_drive_control_pub.publish(msg)
    
    def stop(self):
        msg = TwistStamped()
        msg.twist.linear.x = 0.0
        msg.twist.angular.z = 0.0
        self.diff_drive_control_pub.publish(msg)

    def loop_info_callback(self):
        msg = Bool()
        msg.data = self.is_loop_finished
        self.loop_pub.publish(msg)
        if self.is_loop_finished:
            self.get_logger().info("Başlangıç Noktasına Geri Dönüldü!.")
            self.destroy_node()
            rclpy.shutdown()

def main():
    rclpy.init()
    node = RectangleMotion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == "__main__":
    main()