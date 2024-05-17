import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import UnlessCondition
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import HasNodeParams

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('slam_params_file')

    default_params_file = "/ros2/carbi/src/carbi_bridge/config/mapper_params_online_async.yaml" #os.path.join(get_package_share_directory("articubot_one"), 'config', 'mapper_params_online_async.yaml')
    # default_params_file = "/home/hbbeep/CARBI-1/carbi/src/carbi_bridge/config/mapper_params_online_async.yaml"
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

    carbi_bridge = Node(
        package = "carbi_bridge",
        executable = "bridge.py"
    )

    carbi_sim = Node(
        package = "carbi_bridge",
        executable = "carbi_sim.py"
    )

    lidar_interface = Node(
        package = "rplidar_ros",
        executable = "rplidar_composition",
        # parameters=["ros2/carbi/src/carbi_visualization/config/laserscan_filter.yaml"]

    )

    return LaunchDescription([
        carbi_bridge,
        carbi_sim,
        declare_use_sim_time_argument,
        declare_params_file_cmd,
        log_param_change,
        start_async_slam_toolbox_node,
        lidar_interface
    ])