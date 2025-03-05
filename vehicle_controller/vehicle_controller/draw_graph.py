#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

class DrawGrapNode(Node):
    def __init__(self):
        super().__init__("draw_graph")
        self.files_path = "/home/tahir/midterm_project"
        self.get_files()

    def get_files(self):
        for filename in os.listdir(self.files_path):
            if filename.endswith(".xlsx"):
                file_path = os.path.join(self.files_path, filename)
                name, uznati = os.path.splitext(file_path)
                name = name.split("/")[-1].replace("_", " ")
                print(f"Filename: {file_path}" )
                df = pd.read_excel(file_path)
                self.draw_graph(df, name)

    def draw_graph(self, df, name):
        rmse_x = df["error_x_square"].iloc[-1]
        rmse_y = df["error_y_square"].iloc[-1]

        plt.figure(figsize=(12,6)) # Width

        plt.subplot(1, 2, 1)
        plt.plot(df["time"], df['pose_x'], color="tab:blue", label="real_pose")
        plt.plot(df["time"], df['estimated_pose_x'], color='tab:red', label="estimated_pose")

        plt.xlabel('Timestamp', fontsize=12)
        plt.ylabel('Pose-x', fontsize=12)
        
        plt.legend(loc='upper right', fontsize=10, shadow=True, framealpha=1.0)

        plt.text(-0.05, 1.12, f"RMSE: {rmse_x}", 
         transform=plt.gca().transAxes, fontsize=10, color='black', 
         ha='left', va='top', bbox=dict(facecolor='white', alpha=0.7, edgecolor='black'))

        plt.title(f'{name} yöntemi x ekseni hareket grafiği', fontsize=14, fontweight="bold")
        plt.grid(True, linestyle="--", alpha=0.7)

        plt.xticks(ticks=np.arange(0, df["time"].max(), step=2.0))
        plt.yticks(ticks=np.arange(0, df["pose_x"].max(), step=0.3))

        plt.subplot(1, 2, 2)
        plt.xlabel('Timestamp', fontsize=12)
        plt.ylabel('Pose-Y', fontsize=12)

        plt.plot(df["time"], df['pose_y'], color="tab:blue", label="real_pose")
        plt.plot(df["time"], df['estimated_pose_y'], color='tab:red', label="estimated_pose")

        plt.legend(loc='upper right', fontsize=10, shadow=True, framealpha=1.0)

        plt.text(-0.05, 1.12, f"RMSE: {rmse_y}", 
         transform=plt.gca().transAxes, fontsize=10, color='black', 
         ha='left', va='top', bbox=dict(facecolor='white', alpha=0.7, edgecolor='black'))

        plt.title(f'{name} yöntemi y ekseni hareket grafiği', fontsize=14, fontweight="bold")
        plt.grid(True, linestyle="--", alpha=0.7)

        plt.xticks(ticks=np.arange(0, df["time"].max(), step=2.0))
        plt.yticks(ticks=np.arange(0, df["pose_y"].max(), step=0.3))
    
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = DrawGrapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()