import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("vehicle_description"),
            "launch",
            "gazebo.launch.py"
        ),
        
    )

    imu_logger_node = Node(
        package="vehicle_controller",
        executable="imu_logger"
    )

    return LaunchDescription([
        gazebo,
        imu_logger_node
    ])