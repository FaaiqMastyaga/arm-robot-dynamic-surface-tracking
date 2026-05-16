import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('surface_tracking_controller'),
        'config',
        'controller_config.yaml'
    )

    filter_type_arg = DeclareLaunchArgument(
        'filter_type',
        default_value='kalman_filter',
        choices=['raw', 'ema_filter', 'kalman_filter'],
        description='Filtering method for velocity estimation (raw, ema_filter, kalman_filter)'
    )

    controller_type_arg = DeclareLaunchArgument(
        'controller_type',
        default_value='pid_ff',
        choices=['pid', 'pid_ff', 'mpc'],
        description='Type of controller to use (pid, pid_ff, mpc)'
    )

    controller_node = Node(
        package='surface_tracking_controller',
        executable='dynamic_tracker',
        name='dynamic_tracking',
        output='screen',
        remappings=[
            ('/target_twist', ['/estimated_target_twist/', LaunchConfiguration('filter_type')]),
        ],
        parameters=[
            config_file,
            {'controller_type': LaunchConfiguration('controller_type')}
        ]
    )

    return LaunchDescription([
        filter_type_arg,
        controller_type_arg,
        controller_node
    ])