#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import pandas as pd
import matplotlib.pyplot as plt

class DrawGrapNode(Node):
    def __init__(self):
        super().__init__("draw_graph")
        self.draw_graph()

    def draw_graph(self):
        # Excel dosyasını oku
        df = pd.read_excel('imu_saved_data_all_with_gps_noise.xlsx')
        #df = pd.read_excel('imu_saved_data_wheel_encoder.xlsx')
        #df = pd.read_excel('imu_saved_data1_gps_no_noise.xlsx')

        # Grafik çiz
        plt.plot(df["time"], df['pose_x'], color="tab:blue", label="real_pose")
        plt.plot(df["time"], df['estimated_pose_x'], color='tab:red', label="estimated_pose")
        plt.xlabel('Timestamp')
        plt.ylabel('Pose-x')
        plt.title('Aracın x ekseni hareket grafiği')
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = DrawGrapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()