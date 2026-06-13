import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    experiment_config_file = os.path.join(
        get_package_share_directory('surface_tracking_bringup'),
        'config',
        'experiment_config.yaml'
    )

    controller_config_file = os.path.join(
        get_package_share_directory('surface_tracking_controller'),
        'config',
        'controller_config.yaml'
    )

    # 1. Load the YAML files into separate dictionaries
    with open(experiment_config_file, 'r') as f:
        experiment_yaml_data = yaml.safe_load(f)

    with open(controller_config_file, 'r') as f:
        controller_yaml_data = yaml.safe_load(f)
    
    # 2. Extract parameters from their respective dictionaries
    experiment_params = experiment_yaml_data['global_experiment_manager']['ros__parameters']
    controller_params = controller_yaml_data['/**']['ros__parameters']

    # 3. Extract the horizon to sync the lookahead window
    mpc_horizon = controller_params['mpc_horizon']

    trajectory_generator_node = Node(
        package='surface_tracking_planner',
        executable='trajectory_generator',
        name='trajectory_backend_server',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[
            experiment_params,
            {'lookahead_window': int(mpc_horizon)}
        ]
    )

    return LaunchDescription([
        trajectory_generator_node,
    ])