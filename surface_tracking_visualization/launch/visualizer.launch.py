import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('surface_tracking_aligner'),
        'config',
        'marker_config.yaml'
    )

    active_camera_arg = LaunchConfiguration('active_camera')
    active_target_arg = LaunchConfiguration('active_target')

    camera_visualizer_node = Node(
        package='surface_tracking_visualization',
        executable='camera_visualizer.py',
        name=active_camera_arg,
        parameters=[config_dir],
        output='screen',
        respawn=True,
        respawn_delay=2.0
    )

    target_visualizer_node = Node(
        package='surface_tracking_visualization',
        executable='target_visualizer.py',
        name=active_target_arg,
        parameters=[config_dir],
        output='screen',
        respawn=True,
        respawn_delay=2.0
    )

    return LaunchDescription([
        DeclareLaunchArgument('active_camera', default_value='robot_base'),
        DeclareLaunchArgument('active_target', default_value='target_platform'),
        camera_visualizer_node,
        target_visualizer_node,
    ])