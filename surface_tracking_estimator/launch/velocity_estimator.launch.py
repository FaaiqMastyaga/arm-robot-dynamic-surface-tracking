from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Declare Launch Arguments (Allows command-line overrides)
    filter_type_arg = DeclareLaunchArgument(
        'filter_type', default_value='ema',
        description='Type of velocity filter to use (ema or kalman)'
    )
    
    ema_alpha_arg = DeclareLaunchArgument(
        'ema_alpha', default_value='0.2',
        description='Smoothing factor for the EMA filter (0.0 to 1.0)'
    )
    
    base_frame_arg = DeclareLaunchArgument(
        'base_frame', default_value='elfin_base_link',
        description='The static reference frame (robot base)'
    )
    
    target_frame_arg = DeclareLaunchArgument(
        'target_frame', default_value='whiteboard',
        description='The moving frame to track'
    )
    
    update_rate_arg = DeclareLaunchArgument(
        'update_rate', default_value='100.0',
        description='Filter update rate in Hz'
    )

    # 2. Define the Node
    estimator_node = Node(
        package='surface_tracking_estimator',
        executable='velocity_estimator',
        name='velocity_estimator',
        output='screen',
        parameters=[{
            'filter_type': LaunchConfiguration('filter_type'),
            'ema_alpha': LaunchConfiguration('ema_alpha'),
            'base_frame': LaunchConfiguration('base_frame'),
            'target_frame': LaunchConfiguration('target_frame'),
            'update_rate': LaunchConfiguration('update_rate'),
        }]
    )

    # 3. Return the LaunchDescription
    return LaunchDescription([
        filter_type_arg,
        ema_alpha_arg,
        base_frame_arg,
        target_frame_arg,
        update_rate_arg,
        estimator_node
    ])