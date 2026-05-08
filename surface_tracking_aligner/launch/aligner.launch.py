import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_whiteboard_tf(context, *args, **kwargs):
    # 1. Resolve LaunchConfigurations into actual Python strings
    tool_placement = LaunchConfiguration('tool_placement').perform(context)
    active_target = LaunchConfiguration('active_target').perform(context)

    # 2. Apply conditional logic for the TF arguments
    if tool_placement == 'perpendicular':
        tf_args = ['0.0', '0.005', '-0.1086', '0.0', '0.0', '1.57']
    elif tool_placement == 'parallel':
        tf_args = ['0.0', '-0.1043', '-0.01', '0.0', '0.0', '0.0']
    else:
        print(f"[WARNING] Invalid tool_placement '{tool_placement}'. Defaulting to perpendicular.")
        tf_args = ['0.0', '0.005', '-0.1086', '0.0', '0.0', '1.57']

    # 3. Append the frame names to the end of the argument list
    tf_args.extend([f"{active_target}_aligned", 'whiteboard'])

    # 4. Return the dynamically generated Node
    return [Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='whiteboard_tf_publisher',
        arguments=tf_args,
    )]

def generate_launch_description():
    # --- Paths ---
    aligner_pkg_dir = get_package_share_directory('surface_tracking_aligner')
    bringup_pkg_dir = get_package_share_directory('surface_tracking_bringup')
    
    config_dir = os.path.join(aligner_pkg_dir, 'config', 'marker_config.yaml')
    experiment_config_path = os.path.join(bringup_pkg_dir, 'config', 'experiment_config.yaml')

    # --- Read YAML for Default Value ---
    # This automatically reads your YAML file so you don't have to pass CLI arguments
    default_placement = 'perpendicular'
    try:
        with open(experiment_config_path, 'r') as f:
            yaml_data = yaml.safe_load(f)
            default_placement = yaml_data['global_experiment_manager']['ros__parameters']['tool_placement']
    except Exception as e:
        print(f"[WARNING] Could not parse experiment_config.yaml from bringup package. Using default: {default_placement}")

    # --- Launch Configurations ---
    active_camera_arg = LaunchConfiguration('active_camera')
    active_target_arg = LaunchConfiguration('active_target')

    # --- Nodes ---
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

    return LaunchDescription([
        DeclareLaunchArgument('active_camera', default_value='robot_base'),
        DeclareLaunchArgument('active_target', default_value='target_platform'),
        DeclareLaunchArgument('tool_placement', default_value=default_placement),
        robot_base_node,
        target_platform_node,
        OpaqueFunction(function=generate_whiteboard_tf)
    ])