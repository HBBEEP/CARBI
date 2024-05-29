#!/usr/bin/python3
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch.conditions import UnlessCondition

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import HasNodeParams

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

    ######
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('slam_params_file')

    default_params_file = "/ros2/carbi/src/carbi_visualization/config/mapper_params_online_async.yaml" #os.path.join(get_package_share_directory("articubot_one"), 'config', 'mapper_params_online_async.yaml')
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')
    declare_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')
    has_node_params = HasNodeParams(source_file=params_file, node_name='slam_toolbox')
    actual_params_file = PythonExpression(['"', params_file, '" if ', has_node_params,
                                           ' else "', default_params_file, '"'])
    
    log_param_change = LogInfo(msg=['provided params_file ',  params_file,
                                    ' does not contain slam_toolbox parameters. Using default: ',
                                    default_params_file], condition=UnlessCondition(has_node_params))


    start_async_slam_toolbox_node = Node(
        parameters=[
          actual_params_file,
          {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')

    ######

    return LaunchDescription([
        model_arg,
        use_sim_time_arg,
        robot_state_publisher_node,
        rviz_node,
        # laser_filter_node
        declare_use_sim_time_argument,
        declare_params_file_cmd,
        log_param_change,
        start_async_slam_toolbox_node,
    ])