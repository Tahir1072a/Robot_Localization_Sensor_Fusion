import os
from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    vehicle_localization_dir = get_package_share_directory("vehicle_localization")

    # Static Transform Publisher for GPS sensor
    gps_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='gps_transform',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'vehicle/base_footprint/gps']
    )

    navsat_transform = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform",
        output="screen",
        parameters=[os.path.join(vehicle_localization_dir, "config", "ekf_all.yaml"),{"use_sim_time": True}],
        remappings=[
            ("gps/fix", "/navsat/fix"),
            ("imu", "/imu1")
        ]
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="all_multiple_imu_basic_mean_2",
        output='screen',
        parameters=[os.path.join(vehicle_localization_dir, "config", "ekf_all.yaml")]
    )

    return LaunchDescription([
        gps_transform,
        navsat_transform,
        ekf_node
    ])