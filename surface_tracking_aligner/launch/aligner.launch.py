import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('surface_tracking_aligner'),
        'config',
        'marker_config.yaml'
    )

    camera_aligner_node = Node(
        package='surface_tracking_aligner',
        executable='rigid_body_aligner',
        name='camera_aligner',
        parameters=[config_dir],
        output='screen',
        respawn=True,
        respawn_delay=2.0
    )

    target_aligner_node = Node(
        package='surface_tracking_aligner',
        executable='rigid_body_aligner',
        name='target_aligner',
        parameters=[config_dir],
        output='screen',
        respawn=True,
        respawn_delay=2.0
    )

    return LaunchDescription([
        camera_aligner_node,
        target_aligner_node,
    ])