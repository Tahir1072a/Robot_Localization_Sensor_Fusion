#!/usr/bin/env python3
import rclpy
import time
import random

from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

from std_msgs.msg import Bool

class SelfControl(Node):
    def __init__(self):
        super().__init__("self_controller")
        self.diff_drive_control_pub = self.create_publisher(TwistStamped, "/diff_drive_robot_controller/cmd_vel", 10)

        self.odom_sub = self.create_subscription(Odometry, "/diff_drive_robot_controller/odom", self.odom_callback, 10)
        self.loop_pub = self.create_publisher(Bool, "loop_info", 10)
        self.frequency = 0.05
        self.timer = self.create_timer(self.frequency, self.loop_info_callback)
        self.is_loop_finished = False

        self.speed = 1.5
        self.distance_to_travel = 3.5
        self.current_distance = 0.0
        self.start_position = None
        self.is_moving_forward = True

    def odom_callback(self, msg):
        if self.start_position is None:
            self.start_position = msg.pose.pose.position
            return


        current_position = msg.pose.pose.position
        self.current_distance = abs(current_position.x - self.start_position.x)

        if self.is_moving_forward and self.current_distance >= self.distance_to_travel:
            self.get_logger().info(f"Hedefe varıldı: {self.current_distance} metre")
            self.stop_vehicle()
            time.sleep(3)
            self.is_moving_forward = False
        elif self.is_moving_forward:
            self.move_forward()
        elif not self.is_moving_forward and self.current_distance <= 0.1:
            self.get_logger().info("Araç başlangıç noktasına geri döndü.")
            self.stop_vehicle()
            time.sleep(3)
            self.is_loop_finished = True
        elif not self.is_moving_forward:
            self.move_backward()

    def move_forward(self):
        msg = TwistStamped()
        msg.twist.linear.x = self.speed
        msg.twist.angular.z = 0.0
        self.diff_drive_control_pub.publish(msg)
        #self.get_logger().info("Araç ileri gidiyor!")

    def move_backward(self):
        msg = TwistStamped()
        msg.twist.linear.x = -self.speed
        msg.twist.angular.z = 0.0
        self.diff_drive_control_pub.publish(msg)
        #self.get_logger().info("Araç geri gidiyor!")
    
    def stop_vehicle(self):
        msg = TwistStamped()
        msg.twist.linear.x = 0.0
        msg.twist.angular.z = 0.0
        self.diff_drive_control_pub.publish(msg)
        self.get_logger().info("Araç Durdu!")

    def loop_info_callback(self):
        msg = Bool()
        msg.data = self.is_loop_finished
        self.loop_pub.publish(msg)
        if self.is_loop_finished == True:
            rclpy.shutdown()
            self.destroy_node()


def main():
    rclpy.init()
    node = SelfControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()