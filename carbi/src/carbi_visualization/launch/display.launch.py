#!/usr/bin/python3
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    robot_description_dir = get_package_share_directory("carbi_visualization")

    model_arg = DeclareLaunchArgument(name="model", default_value=os.path.join(
                                        robot_description_dir, "description", "robot.urdf.xacro"))

    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]),
                                       value_type=str)

    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="false",
        description="Use simulation (ROS time) instead of real time"
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, 'use_sim_time': LaunchConfiguration('use_sim_time')} ]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(robot_description_dir, "rviz", "display.rviz")],
    )

    laser_filter_node = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        name="laser_filter",
        parameters=["ros2/carbi/src/carbi_visualization/config/laserscan_filter.yaml"]
    )


    return LaunchDescription([
        model_arg,
        use_sim_time_arg,
        robot_state_publisher_node,
        rviz_node,
        laser_filter_node
    ])