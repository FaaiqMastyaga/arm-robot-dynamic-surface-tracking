import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('surface_tracking_estimator'),
        'config',
        'estimator_config.yaml'
    )

    estimator_node = Node(
        package='surface_tracking_estimator',
        executable='velocity_estimator',
        name='velocity_estimator',
        output='screen',
        parameters=[config_file]
    )

    return LaunchDescription([
        estimator_node
    ])