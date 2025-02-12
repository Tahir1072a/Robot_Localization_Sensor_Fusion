#!/usr/bin/env python3
import rclpy
import subprocess

from rclpy.node import Node

from nav_msgs.msg import Odometry 
from sensor_msgs.msg import Imu

class ImuLogger(Node):
    def __init__(self):
        super().__init__("imu_logger")

        self.file_path = "imu_data.csv"
        subprocess.run(["code", self.file_path])      

        self.imu_file = open(self.file_path, "w")
        self.imu_file.write(f"{'time':<16} {'pose_x':<16} {'pose_y':<16} {'pose_z':<16} {'ax1':<16} {'ay1':<16} {'az1':<16} {'gx1':<16} {'gy1':<16} {'gz1':<16} {'ax2':<16} {'ay2':<16} {'az2':<16} {'gx2':<16} {'gy2':<16} {'gz2':<16} {'ax3':<16} {'ay3':<16} {'az3':<16} {'gx3':<16} {'gy3':<16} {'gz3':<16}\n")

        self.buffer = {}
        self.timestamp_arr = []

        self.imu1_sub = self.create_subscription(Imu, "/imu1", lambda msg: self.imu_sub_callback(msg, "imu1"), 10)
        self.imu2_sub = self.create_subscription(Imu, "/imu2", lambda msg: self.imu_sub_callback(msg, "imu2"), 10)
        self.imu3_sub = self.create_subscription(Imu, "/imu3", lambda msg: self.imu_sub_callback(msg, "imu3"), 10)

        self.odom_sub = self.create_subscription(Odometry, "/diff_drive_robot_controller/odom", self.odom_callback, 10)

    def odom_callback(self, msg):
        current_pos = msg.pose.pose.position
        timestamp_sec = msg.header.stamp.sec
        timestamp_nano = msg.header.stamp.nanosec
        timestamp = f"{timestamp_sec}.{timestamp_nano}"
                
        if timestamp not in self.timestamp_arr:
            self.timestamp_arr.append(timestamp)
            self.buffer[self.timestamp_arr.index(timestamp)] = {}
        
        self.buffer[self.timestamp_arr.index(timestamp)]["pose"] = {
            "pose_x": current_pos.x,
            "pose_y": current_pos.y,
            "pose_z": current_pos.z
        }

    def imu_sub_callback(self, msg, imu_id):
        timestamp_nano = msg.header.stamp.nanosec
        timestamp_sec = msg.header.stamp.sec
        timestamp = f"{timestamp_sec}.{timestamp_nano}"

        if timestamp not in self.timestamp_arr:
            self.timestamp_arr.append(timestamp)
            self.buffer[self.timestamp_arr.index(timestamp)] = {}
        
        self.buffer[self.timestamp_arr.index(timestamp)][imu_id] = {
            "ax": msg.linear_acceleration.x,
            "ay": msg.linear_acceleration.y,
            "az": msg.linear_acceleration.z,
            "gx": msg.angular_velocity.x,
            "gy": msg.angular_velocity.y,
            "gz": msg.angular_velocity.z,
        }
        
        if len(self.buffer[self.timestamp_arr.index(timestamp)]) == 4:
            self.write_to_file(timestamp)
            
            timestamp_index = self.timestamp_arr.index(timestamp)
            del self.buffer[timestamp_index]
            self.timestamp_arr.remove(timestamp)
    
    def write_to_file(self, timestamp):
        data = self.buffer[self.timestamp_arr.index(timestamp)]

        data_str = f"{timestamp:<15} "
        for imu_id in ["pose", "imu1", "imu2", "imu3"]:
            if imu_id == "pose":
                pass
                pose_data = data[imu_id]
                data_str += f"{pose_data['pose_x']:<15.12f}, {pose_data['pose_y']:<15.12f}, {pose_data['pose_z']:<15.12f}"
            else: 
                imu_data = data[imu_id]
                data_str += f"{imu_data['ax']:<15.12f}, {imu_data['ay']:<15.12f}, {imu_data['az']:<15.12f}, {imu_data['gx']:<15.12f}, {imu_data['gy']:<15.12f}, {imu_data['gz']:<15.12f}, "
        
        data_str = data_str.rstrip(", ") + "\n"
        self.imu_file.write(data_str)
        self.imu_file.flush()


def main():
    rclpy.init()
    node = ImuLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()