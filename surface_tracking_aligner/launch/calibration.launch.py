import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Allow the user to choose what to calibrate via command line
        DeclareLaunchArgument(
            'target', 
            default_value='aligner', 
            description='What to calibrate: "aligner" or "platform"'
        ),
        # Default workspace source path (can be overridden via command line if needed)
        DeclareLaunchArgument(
            'ws_src', 
            default_value='/home/dian/faaiq/Tugas_Akhir/dynamic_surface_tracking_ws/src', 
            description='Path to the workspace src directory'
        ),
        OpaqueFunction(function=launch_setup)
    ])

def launch_setup(context, *args, **kwargs):
    # 1. Get the values from the command line arguments
    target_type = LaunchConfiguration('target').perform(context)
    ws_src = LaunchConfiguration('ws_src').perform(context)

    # Base repository directory
    repo_dir = os.path.join(ws_src, 'dynamic_surface_tracking')

    # 2. Path for Marker Config (where we WRITE data) -> surface_tracking_aligner
    aligner_pkg_dir = os.path.join(repo_dir, 'surface_tracking_aligner')
    marker_config_path = os.path.join(aligner_pkg_dir, 'config', 'marker_config.yaml')

    # 3. Path for Experiment Config (where we READ data) -> surface_tracking_bringup
    bringup_pkg_dir = os.path.join(repo_dir, 'surface_tracking_bringup')
    experiment_config_path = os.path.join(bringup_pkg_dir, 'config', 'experiment_config.yaml')

    # 4. Read the global experiment config
    with open(experiment_config_path, 'r') as f:
        exp_data = yaml.safe_load(f)
    
    params = exp_data['global_experiment_manager']['ros__parameters']

    # 5. Dynamically figure out the tool name based on the config
    if target_type == 'aligner':
        tool_name = params['active_camera_aligner']  # e.g., 'robot_base'
    elif target_type == 'platform':
        base = params['active_target_platform']
        config = params['marker_configuration']
        num = params['number_of_markers']
        tool_name = f"{base}_{config}_{num}pt"       # e.g., 'target_platform_coplanar_5pt'
    else:
        raise ValueError("Launch argument 'target' must be either 'aligner' or 'platform'")

    # 6. Dynamically find the .aimtool file in the user's home directory
    home_dir = os.path.expanduser('~')
    aimtool_path = os.path.join(home_dir, '.aimooe', 'tools', 'surface_tracking', f'{tool_name}.aimtool')

    print(f"\n--- CALIBRATION SETUP ---")
    print(f"Target Tool: {tool_name}")
    print(f"Reading Global Config: {experiment_config_path}")
    print(f"Aimtool Path: {aimtool_path}")
    print(f"Modifying Marker YAML: {marker_config_path}\n")

    # 7. Create the Calibration Node
    calibration_node = Node(
        package='surface_tracking_aligner',
        executable='calibration.py',
        name='calibration',
        output='screen',
        parameters=[{
            'yaml_path': marker_config_path,
            'target_node_name': tool_name,
            'aimtool_path': aimtool_path
        }]
    )

    return [calibration_node]