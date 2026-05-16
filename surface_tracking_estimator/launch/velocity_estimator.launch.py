import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('surface_tracking_estimator'),
        'config',
        'estimator_config.yaml'
    )

    target_frame_arg = DeclareLaunchArgument(
        'target_frame',
        default_value='whiteboard',
        description='The frame to which the velocity is estimated'
    )

    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='elfin_base_link',
        description='The base frame for the velocity estimation'
    )

    estimator_node = Node(
        package='surface_tracking_estimator',
        executable='velocity_estimator',
        name='velocity_estimator',
        output='screen',
        parameters=[
            config_file,
            {
                'target_frame': LaunchConfiguration('target_frame'),
                'base_frame': LaunchConfiguration('base_frame')
            }
        ]
    )

    return LaunchDescription([
        target_frame_arg,
        base_frame_arg,
        estimator_node
    ])