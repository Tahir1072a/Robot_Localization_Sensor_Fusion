#!/usr/bin/env python3
import rclpy
import subprocess
import shutil
import pandas as pd

from pathlib import Path

from rclpy.node import Node
from threading import Lock, Thread
from queue import Queue

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool

class ImuLogger(Node):
    def __init__(self):
        super().__init__("imu_logger")

        self.df = pd.DataFrame(columns=[
            "time", "pose_x", "pose_y", "pose_z",
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

        self.odom_sub = self.create_subscription(Odometry, "/diff_drive_robot_controller/odom", self.odom_callback, 30)

        self.loop_info_sub = self.create_subscription(Bool, "loop_info", self.loop_info_callback, 5)

    def odom_callback(self, msg):
        current_pos = msg.pose.pose.position
        timestamp = self.get_timestamp(msg)

        if self.buffer.get(timestamp) is None:
            self.buffer[timestamp] = {}

        with self.buffer_lock:
            self.buffer[timestamp]["pose"] = {
                "pose_x": current_pos.x,
                "pose_y": current_pos.y,
                "pose_z": current_pos.z
            }

    def imu_sub_callback(self, msg, imu_id):
        timestamp = self.get_timestamp(msg)

        if self.buffer.get(timestamp) is None:
            self.buffer[timestamp] = {}

        #self.get_logger().info("-----------------------------DEBUG-------------------------------")
        #self.get_logger().info(f"Processing {imu_id} data for timestamp: {timestamp}")
        #self.get_logger().info(str(self.buffer[str_timestamp]))
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
        
        if len(self.buffer[timestamp]) == 4:
            self.prepare_data(timestamp)
    
    def prepare_data(self, timestamp):
        data = self.buffer[timestamp]

        row = {
            "time": timestamp,
            "pose_x": data["pose"]["pose_x"],
            "pose_y": data["pose"]["pose_y"],
            "pose_z": data["pose"]["pose_z"],
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
        return f"{timestamp_sec}.{timestamp_nano}"

    def loop_info_callback(self, msg):
        if msg.data:
            self.is_finished = True

           # CSV dosyasını kaydet
            new_csv_path = "imu_saved_data.csv"
            destination_csv_path = Path(new_csv_path)
            for i in range(1, 10):
                if not destination_csv_path.exists():
                    break
                new_csv_path = f"imu_saved_data{i}.csv"
                destination_csv_path = Path(new_csv_path)

            self.save_dataframe_to_csv_formatted(destination_csv_path)

            #excel dosyasına kaydet
            new_excel_path = "imu_saved_data.xlsx"
            destination_excel_path = Path(new_excel_path)
            for i in range(1, 10):
                if not destination_excel_path.exists():
                    break
                new_excel_path = f"imu_saved_data{i}.xlsx"
                destination_excel_path = Path(new_excel_path)
            self.df.to_excel(destination_excel_path, index=False)

            rclpy.shutdown()
            self.destroy_node()

    def save_dataframe_to_csv_formatted(self, file_path):

        with open(file_path, "w") as f:
            
            header = f"{'time':<15}, {'pose_x':<15}, {'pose_y':<15}, {'pose_z':<15}, "
            for imu_id in ["1", "2", "3"]:
                header += f"{'ax' + imu_id:<15}, {'ay' + imu_id:<15}, {'az' + imu_id:<15}, "
                header += f"{'gx' + imu_id:<15}, {'gy' + imu_id:<15}, {'gz' + imu_id:<15} "
            header = header.rstrip() + "\n"
            f.write(header)
            
            for _, row in self.df.iterrows():
                data_str = f"{row['time']:<15}, {row['pose_x']:<15.12f}, {row['pose_y']:<15.12f}, {row['pose_z']:<15.12f}, "
                for imu_id in ["1", "2", "3"]:
                    data_str += f"{row[f'ax{imu_id}']:<15.12f}, {row[f'ay{imu_id}']:<15.12f}, {row[f'az{imu_id}']:<15.12f}, "
                    data_str += f"{row[f'gx{imu_id}']:<15.12f}, {row[f'gy{imu_id}']:<15.12f}, {row[f'gz{imu_id}']:<15.12f} "
                data_str = data_str.rstrip() + "\n"
                f.write(data_str)

    
def main():
    rclpy.init()
    node = ImuLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()