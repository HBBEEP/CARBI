import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import UnlessCondition
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import HasNodeParams

def generate_launch_description():
    
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
	parameters = [ {
               #  'inverted': True,
                'angle_compensate': True,
#                'flip_x_axis': True,
                'use_sim_time': False}]
    )

    robot_localization = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[os.path.join(get_package_share_directory("carbi_bridge"), 'config', 'ekf.yaml')],
    )

    return LaunchDescription([
        carbi_bridge,
#        carbi_sim,
        # declare_use_sim_time_argument,
        # declare_params_file_cmd,
        # log_param_change,
        # start_async_slam_toolbox_node,
        lidar_interface,
#	robot_localization
    ])
