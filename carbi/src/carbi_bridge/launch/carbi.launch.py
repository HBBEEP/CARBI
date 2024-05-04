from launch import LaunchDescription
from launch_ros.actions import Node


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
        executable = "rplidar_composition"
    )
    
    return LaunchDescription([
        carbi_bridge,
        carbi_sim
        # lidar_interface
    ])