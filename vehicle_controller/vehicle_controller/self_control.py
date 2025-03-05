#!/usr/bin/env python3
import rclpy
import time

from tf_transformations import euler_from_quaternion

from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from tf2_msgs.msg import TFMessage

from std_msgs.msg import Bool

class SelfControl(Node):
    def __init__(self):
        super().__init__("self_controller")
        self.diff_drive_control_pub = self.create_publisher(TwistStamped, "/diff_drive_robot_controller/cmd_vel", 10)

        self.vehicle_pose_sub = self.create_subscription(TFMessage, "/vehicle/real_pose", self.vehicle_pose_callback, 10)
        self.loop_pub = self.create_publisher(Bool, "loop_info", 10)
        self.frequency = 0.05
        self.timer = self.create_timer(self.frequency, self.loop_info_callback)
        self.is_loop_finished = False

        self.speed = 1.0
        self.distance_to_travel = 15.0
        self.current_distance = 0.0
        self.start_position = None
        self.is_moving_forward = True
        self.is_turn = False
        self.is_moving_backward = False

        self.current_yaw = 0.0
        self.target_yaw = 3.14

    def vehicle_pose_callback(self, msg):
        if self.start_position is None:
            self.start_position = msg.transforms[0].transform.translation
            return


        current_position = msg.transforms[0].transform.translation
        self.current_distance = abs(current_position.x - self.start_position.x)
        orientation_list = [msg.transforms[0].transform.rotation.x, msg.transforms[0].transform.rotation.y, msg.transforms[0].transform.rotation.z, msg.transforms[0].transform.rotation.w]
        _, _, self.current_yaw = euler_from_quaternion(orientation_list)

        #self.get_logger().info(f"Current Distance: {self.current_distance} Current Yaw: {self.current_yaw}")

        if self.is_moving_forward and self.current_distance >= self.distance_to_travel:
            self.get_logger().info(f"Hedefe varildi: {self.current_distance} metre")
            self.stop_vehicle()
            time.sleep(1.0)
            self.is_moving_forward = False
            self.is_turn = True
            self.get_logger().info("Araç dönüşe başladi.")
        elif self.is_moving_forward:
            self.move_forward()
        elif self.target_yaw - self.current_yaw <= 0.005 and not self.is_moving_forward and not self.is_moving_backward:
            self.stop_vehicle()
            self.is_turn = False
            self.is_moving_backward = True
        elif self.is_turn:
            self.turn_vehicle()
        elif not self.is_moving_forward and self.current_distance <= 0.1:
            self.stop_vehicle()
            time.sleep(1.0)
            self.get_logger().info("Araç başlangiç noktasina geri döndü.")
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
        msg.twist.linear.x = self.speed
        msg.twist.angular.z = 0.0
        self.diff_drive_control_pub.publish(msg)
        #self.get_logger().info("Araç geri gidiyor!")
    
    def stop_vehicle(self):
        msg = TwistStamped()
        msg.twist.linear.x = 0.0
        msg.twist.angular.z = 0.0
        self.diff_drive_control_pub.publish(msg)
        self.get_logger().info("Araç Durdu!")

    def turn_vehicle(self):
        msg = TwistStamped()
        msg.twist.linear.x = 0.0
        msg.twist.angular.z = 0.4
        self.diff_drive_control_pub.publish(msg)
        #self.get_logger().info("Araç dönüyor!")

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