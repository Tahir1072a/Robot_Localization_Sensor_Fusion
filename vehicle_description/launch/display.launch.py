from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    vehicle_description_dir = get_package_share_directory("vehicle_description")

    model_arg = DeclareLaunchArgument(name="model", default_value=os.path.join(vehicle_description_dir, "urdf", "vehicle_core.urdf.xacro"))
    vehicle_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]))

    vehicle_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": vehicle_description}]
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(vehicle_description_dir, "rviz", "display.rviz")]
    )

    return LaunchDescription([
        model_arg,
        joint_state_publisher_gui_node,
        vehicle_state_publisher_node,
        rviz2
    ])