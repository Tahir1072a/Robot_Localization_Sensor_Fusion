import os
from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    vehicle_localization_dir = get_package_share_directory("vehicle_localization")

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(vehicle_localization_dir, "config", "ekf.yaml")]
    )

    return LaunchDescription([
        ekf_node
    ])