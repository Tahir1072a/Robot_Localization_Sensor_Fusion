from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os
from os import pathsep
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    vehicle_description_dir = get_package_share_directory("vehicle_description")

    model_arg = DeclareLaunchArgument(name="model", default_value=os.path.join(vehicle_description_dir, "urdf", "vehicle_core.urdf.xacro"))
    world_name_arg = DeclareLaunchArgument(name="world_name", default_value="empty")

    world_path = PathJoinSubstitution([
        vehicle_description_dir,
        "worlds",
        PythonExpression(expression=["'", LaunchConfiguration("world_name"), "'", " + '.world'"])
    ])

    model_path = str(Path(vehicle_description_dir).parent.resolve())
    model_path += pathsep + os.path.join(get_package_share_directory("vehicle_description"), "models")

    gazebo_resource_path = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        model_path
    )

    vehicle_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]), value_type=str)

    vehicle_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": vehicle_description, "use_sim_time": True}]
    )

    vehicle_controllers_yaml = PathJoinSubstitution(
        [
            vehicle_description_dir,
            "config",
            "vehicle_controller.yaml"
        ]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"
        ]),
        launch_arguments={
            "gz_args": PythonExpression(["'", world_path, " -v 4 -r'"])
        }.items()
    )

    gz_spawn_enttiy = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description", "-name", "vehicle"]
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/imu1@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/imu2@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/imu3@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/model/vehicle/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/navsat/fix@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat"
        ],
        remappings=[
            ("model/vehicle/pose", "/vehicle/real_pose")
        ]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"]
    )

    vehicle_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_robot_controller", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[vehicle_controllers_yaml],
        output=["both"],
        remappings=[
            ("~/robot_description", "/robot_description"),
            ("/diffbot_base_controller/cmd_vel", "/cmd_vel"),
        ]
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(vehicle_description_dir, "rviz" , "display.rviz")]
    )

    return LaunchDescription([
        model_arg,
        world_name_arg,
        gazebo_resource_path,
        vehicle_state_publisher_node,
        gazebo,
        gz_spawn_enttiy,
        gz_ros2_bridge,
        control_node,
        vehicle_controller_spawner,
        joint_state_broadcaster_spawner,
        rviz2
    ])