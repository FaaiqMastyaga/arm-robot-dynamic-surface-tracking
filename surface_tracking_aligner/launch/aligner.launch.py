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

    robot_base_node = Node(
        package='surface_tracking_aligner',
        executable='rigid_body_aligner',
        name=active_camera_arg,
        parameters=[config_dir],
        output='screen',
        respawn=True,
        respawn_delay=2.0
    )

    target_platform_node = Node(
        package='surface_tracking_aligner',
        executable='rigid_body_aligner',
        name=active_target_arg,
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
            [active_target_arg, '_aligned'], 'whiteboard'
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('active_camera', default_value='robot_base'),
        DeclareLaunchArgument('active_target', default_value='target_platform'),
        robot_base_node,
        target_platform_node,
        whiteboard_tf_node
    ])