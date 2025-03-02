#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import os
import pandas as pd
import matplotlib.pyplot as plt

class DrawGrapNode(Node):
    def __init__(self):
        super().__init__("draw_graph")
        self.files_path = "/home/tahir/midterm_project"
        self.get_files()
        self.names = [""]

    def get_files(self):
        for filename in os.listdir(self.files_path):
            if filename.endswith(".xlsx"):
                file_path = os.path.join(self.files_path, filename)
                print(f"Filename: {file_path}" )
                df = pd.read_excel(file_path)
                self.draw_graph(df)

    def draw_graph(self, df):
        rmse_x = df["error_x_square"].iloc[-1]
        print(rmse_x)
        plt.figure(figsize=(12,6)) # Width

        plt.plot(df["time"], df['pose_x'], color="tab:blue", label="real_pose")
        plt.plot(df["time"], df['estimated_pose_x'], color='tab:red', label="estimated_pose")

        plt.xlabel('Timestamp', fontsize=12)
        plt.ylabel('Pose-x', fontsize=12)
        
        plt.legend(loc='upper right', fontsize=10, shadow=True, framealpha=1.0)

        plt.text(0.25, 0.95, f"RMSE: {rmse_x}", 
         transform=plt.gca().transAxes, fontsize=10, color='black', 
         ha='right', va='top', bbox=dict(facecolor='white', alpha=0.7, edgecolor='black'))

        plt.title('Aracin x ekseni hareket grafiği', fontsize=14, fontweight="bold")
        plt.grid(True, linestyle="--", alpha=0.5)
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = DrawGrapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()