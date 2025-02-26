import os
from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    vehicle_localization_dir = get_package_share_directory("vehicle_localization")

    navsat_transform = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform",
        output="screen",
        parameters=[os.path.join(vehicle_localization_dir, "config", "ekf_with_gps.yaml"),{"use_sim_time": True}],
        remappings=[
            ("gps/fix", "/navsat/fix"),
            ("imu", "/imu1")
        ]
    )

    ekf_node_local = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        output='screen',
        parameters=[os.path.join(vehicle_localization_dir, "config", "ekf_with_gps.yaml")]
    )

    ekf_node_global = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_map",
        output='screen',
        parameters=[os.path.join(vehicle_localization_dir, "config", "ekf_with_gps.yaml")]
    )

    return LaunchDescription([
        navsat_transform,
        ekf_node_global
    ])