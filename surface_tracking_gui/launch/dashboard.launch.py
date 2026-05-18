import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Whether to use simulation time (Gazebo) or real time (ROS clock)'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    dashboard_server = Node(
        package='surface_tracking_gui',
        executable='dashboard_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
        respawn=True,
        respawn_delay=2.0
    )

    dashboard_client = Node(
        package='surface_tracking_gui',
        executable='dashboard_client',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
        respawn=True,
        respawn_delay=2.0
    )

    return LaunchDescription([
        use_sim_time_arg,
        dashboard_server,
        dashboard_client
    ])