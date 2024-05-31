import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_file = "ros2/carbi/floor3.yaml"
    param_file = "src/carbi_navigation/config/navigation_param.yaml"

    # map_dir = os.path.join(get_package_share_directory(
    #     'carbi_navigation'), 'maps_show')
    # map_file = LaunchConfiguration('map', default=os.path.join(
    #     map_dir, 'floor3.yaml'))
    # # print(map_file)
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    
    robot_description_dir = get_package_share_directory("carbi_navigation")

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

    return LaunchDescription([
        # DeclareLaunchArgument(
        #     'map',
        #     default_value=map_file,
        #     description='Full path to map file to load'),

        # DeclareLaunchArgument(
        #     'params',
        #     default_value=param_file,
        #     description='Full path to param file to load'),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         [nav2_launch_file_dir, '/bringup_launch.py']),
        #     launch_arguments={
        #         'map': map_file,
        #         'use_sim_time': use_sim_time,
        #         'params_file': param_file}.items(),
        # ),
        use_sim_time_arg,
        model_arg,
        robot_state_publisher_node,
        rviz_node,
        ])