import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('surface_tracking_bringup'),
        'config',
        'experiment_config.yaml'
    )

    with open(config_file, 'r') as f:
        yaml_data = yaml.safe_load(f)
    
    experiment_params = yaml_data['global_experiment_manager']['ros__parameters']

    trajectory_generator_node = Node(
        package='surface_tracking_planner',
        executable='trajectory_generator',
        name='trajectory_backend_server',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[experiment_params]
    )

    return LaunchDescription([
        trajectory_generator_node,
    ])