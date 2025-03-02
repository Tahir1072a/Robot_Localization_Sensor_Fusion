import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
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

    ekf_filter = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("vehicle_localization"),
            "launch",
            "ekf_filter.launch.py"
        )
    )

    self_control = Node(
        package="vehicle_controller",
        executable="self_control"
    )

    delay_self_control = TimerAction(
        period=1.0,
        actions=[self_control]
    )

    return LaunchDescription([
        gazebo,
        imu_logger_node,
        ekf_filter,
        delay_self_control
    ])