#!/usr/bin/env python3
import rclpy
import pandas as pd
import numpy as np

from pathlib import Path

from rclpy.node import Node
from threading import Lock, Thread
from queue import Queue

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage

class ImuLogger(Node):
    def __init__(self):
        super().__init__("imu_logger")

        self.df = pd.DataFrame(columns=[
            "time", "pose_x", "pose_y", "pose_z",
            "estimated_pose_x", "estimated_pose_y", "estimated_pose_z",
            "error_x_square", "error_y_square", "error_z_square",
            "ax1", "ay1", "az1", "gx1", "gy1", "gz1",
            "ax2", "ay2", "az2", "gx2", "gy2", "gz2",
            "ax3", "ay3", "az3", "gx3", "gy3", "gz3"
        ])

        self.buffer = {}
        self.buffer_lock = Lock()
        self.queue = Queue()
        self.is_finished = False

        self.w_thread = Thread(target=self.write_to_file, daemon=True)
        self.w_thread.start()

        self.imu1_sub = self.create_subscription(Imu, "/imu1", lambda msg: self.imu_sub_callback(msg, "imu1"), 30)
        self.imu2_sub = self.create_subscription(Imu, "/imu2", lambda msg: self.imu_sub_callback(msg, "imu2"), 30)
        self.imu3_sub = self.create_subscription(Imu, "/imu3", lambda msg: self.imu_sub_callback(msg, "imu3"), 30)

        self.vehicle_pose_sub = self.create_subscription(TFMessage, "/vehicle/real_pose", self.vehicle_real_pose_callback, 30)
        self.ekf_filtered_odom_sub = self.create_subscription(Odometry, "/odometry/filtered", self.ekf_filtered_odom ,30)

        self.loop_info_sub = self.create_subscription(Bool, "loop_info", self.loop_info_callback, 5)

    #Son gelen veri!!
    def ekf_filtered_odom(self, msg):
        estimated_pose = msg.pose.pose.position
        timestamp = self.get_timestamp(msg)
        
        #self.get_logger().info(f"Estimated Pose: {timestamp}")

        if self.buffer.get(timestamp) is None:
            self.buffer[timestamp] = {}
        
        with self.buffer_lock:
            self.buffer[timestamp]["estimated_pose"] = {
                "estimated_pose_x": estimated_pose.x,
                "estimated_pose_y": estimated_pose.y,
                "estimated_pose_z": estimated_pose.z
            }
        
        #self.get_logger().info(str(self.buffer[timestamp]))

        if len(self.buffer[timestamp]) == 5:
            self.prepare_data(timestamp)
            
    def vehicle_real_pose_callback(self, msg):
        current_pose = msg.transforms[0].transform.translation
        timestamp = self.get_timestamp(msg.transforms[0])

        #self.get_logger().info(f"Pose: {timestamp}")
        
        if self.buffer.get(timestamp) is None:
            self.buffer[timestamp] = {}

        with self.buffer_lock:
            self.buffer[timestamp]["pose"] = {
                "pose_x": current_pose.x,
                "pose_y": current_pose.y,
                "pose_z": current_pose.z
            }       

    def imu_sub_callback(self, msg, imu_id):
        timestamp = self.get_timestamp(msg)

        #self.get_logger().info(f"Imu: {timestamp}")

        if self.buffer.get(timestamp) is None:
            self.buffer[timestamp] = {}

        #self.get_logger().info("-----------------------------DEBUG-------------------------------")
        #self.get_logger().info(f"Processing {imu_id} data for timestamp: {timestamp}")
        #self.get_logger().info(str(self.buffer[timestamp]))
        #self.get_logger().info(timestamp)
        #self.get_logger().info(str(len(self.timestamp_arr)))

        with self.buffer_lock:
            self.buffer[timestamp][imu_id] = {
                "ax": msg.linear_acceleration.x,
                "ay": msg.linear_acceleration.y,
                "az": msg.linear_acceleration.z,
                "gx": msg.angular_velocity.x,
                "gy": msg.angular_velocity.y,
                "gz": msg.angular_velocity.z,
            }
            
        """
        if len(self.buffer[timestamp]) == 5:
            self.get_logger().info("Okey")
            self.prepare_data(timestamp)
        """
    
    def prepare_data(self, timestamp):
        data = self.buffer[timestamp]
        error_x, error_y, error_z = self.calculate_squart_err(data)

        row = {
            "time": timestamp,
            "pose_x": data["pose"]["pose_x"],
            "pose_y": data["pose"]["pose_y"],
            "pose_z": data["pose"]["pose_z"],
            "estimated_pose_x": data["estimated_pose"]["estimated_pose_x"],
            "estimated_pose_y": data["estimated_pose"]["estimated_pose_y"],
            "estimated_pose_z": data["estimated_pose"]["estimated_pose_z"],
            "error_x_square": error_x,
            "error_y_square": error_y,
            "error_z_square": error_z
        }

        for imu_id in ["imu1", "imu2", "imu3"]:
            imu_data = data[imu_id]
            row.update({
                f"ax{imu_id[-1]}": imu_data["ax"],
                f"ay{imu_id[-1]}": imu_data["ay"],
                f"az{imu_id[-1]}": imu_data["az"],
                f"gx{imu_id[-1]}": imu_data["gx"],
                f"gy{imu_id[-1]}": imu_data["gy"],
                f"gz{imu_id[-1]}": imu_data["gz"],
            })
        
        with self.buffer_lock:
            self.queue.put(row)
            del self.buffer[timestamp]

    def write_to_file(self):
        while not self.is_finished:
            try:
                row = self.queue.get()
                # pandas 2.0 sürümü ile df'ye satır ekleme yöntemi değişmiştir!
                self.df = pd.concat([self.df, pd.DataFrame([row])], ignore_index=True)

            except Exception as e:
                self.get_logger().error(str(e))
                continue
    
    def get_timestamp(self, msg):
        timestamp_nano = msg.header.stamp.nanosec
        timestamp_sec = msg.header.stamp.sec
        return f"{timestamp_sec}.{timestamp_nano:09d}"

    def loop_info_callback(self, msg):
        if msg.data:
            self.is_finished = True
            rmse_x, rmse_y, rmse_z = self.calculate_RMSE()
            #Son satıra mse değerini ekle!
            index = self.df.index[-1] + 1
            self.df.loc[index, "error_x_square"] = rmse_x
            self.df.loc[index, "error_y_square"] = rmse_y
            self.df.loc[index, "error_z_square"] = rmse_z

            self.save_to_excel_file()
            rclpy.shutdown()
            self.destroy_node()

    def calculate_squart_err(self, data):
        pose_x, pose_y, pose_z = data["pose"]["pose_x"], data["pose"]["pose_y"], data["pose"]["pose_z"]
        estimated_pose_x, estimated_pose_y, estimated_pose_z = data["estimated_pose"]["estimated_pose_x"], data["estimated_pose"]["estimated_pose_y"], data["estimated_pose"]["estimated_pose_z"]
        
        # Bu yöntemi araştır??
        #return ((pose_x - estimated_pose_x) ** 2 + (pose_y - estimated_pose_y) ** 2 + (pose_z - estimated_pose_z) ** 2) / 3
        return (pose_x - estimated_pose_x) ** 2, (pose_y - estimated_pose_y) ** 2, (pose_z - estimated_pose_z) ** 2
    
    """ Root Mean Squared Error"""
    def calculate_RMSE(self):
        # 0 değerleri satırları temizle!
        self.df = self.df[self.df["estimated_pose_x"] != 0]

        errors_x_square, errors_y_square, errors_z_square = self.df["error_x_square"].to_numpy(), self.df["error_y_square"].to_numpy(), self.df["error_z_square"].to_numpy()
        rmse_x, rmse_y, rmse_z = np.sqrt(np.mean(errors_x_square)), np.sqrt(np.mean(errors_y_square)), np.sqrt(np.mean(errors_z_square))
        return rmse_x, rmse_y, rmse_z

    def save_to_excel_file(self):
        #excel dosyasına kaydet
        new_excel_path = "imu_saved_data.xlsx"
        destination_excel_path = Path(new_excel_path)
        for i in range(1, 10):
            if not destination_excel_path.exists():
                break
            new_excel_path = f"imu_saved_data{i}.xlsx"
            destination_excel_path = Path(new_excel_path)
        self.df.to_excel(destination_excel_path, index=False)

def main():
    rclpy.init()
    node = ImuLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()