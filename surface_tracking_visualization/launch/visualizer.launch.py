import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('surface_tracking_calibration'),
        'config',
        'marker_config.yaml'
    )

    camera_visualizer_node = Node(
        package='surface_tracking_visualization',
        executable='camera_visualizer.py',
        name='camera_visualizer',
        parameters=[config_dir],
        output='screen',
        respawn=True,
        respawn_delay=2.0
    )

    target_visualizer_node = Node(
        package='surface_tracking_visualization',
        executable='target_visualizer.py',
        name='target_visualizer',
        parameters=[config_dir],
        output='screen',
        respawn=True,
        respawn_delay=2.0
    )

    whiteboard_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='whiteboard_tf_publisher',
        arguments=[
            '0.01', '0.0', '-0.11', 
            '3.14', '0.0', '1.57', 
            'target_platform_aligned', 'whiteboard'
        ],
    )

    return LaunchDescription([
        camera_visualizer_node,
        target_visualizer_node,
        whiteboard_tf_node
    ])