import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    master_dashboard_node = Node(
        package='surface_tracking_gui',
        executable='master_dashboard',
        output='screen',
        respawn=True,
        respawn_delay=2.0
    )

    return LaunchDescription([
        master_dashboard_node,
    ])