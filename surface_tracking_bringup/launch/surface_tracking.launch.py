import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    bringup_dir = get_package_share_directory('surface_tracking_bringup')
    aimooe_ros2_dir = get_package_share_directory('aimooe_ros2')
    elfin10_l_ros2_moveit2_dir = get_package_share_directory('elfin10_l_ros2_moveit2')
    aligner_dir = get_package_share_directory('surface_tracking_aligner')
    visualization_dir = get_package_share_directory('surface_tracking_visualization')

    global_yaml_path = os.path.join(bringup_dir, 'config', 'experiment_config.yaml')

    with open(global_yaml_path, 'r') as file:
        global_config = yaml.safe_load(file)['global_experiment_manager']['ros__parameters']

    number_of_markers = global_config['number_of_markers']
    marker_configuration = global_config['marker_configuration']
    active_camera = global_config['active_camera_aligner']
    active_target = f"{global_config['active_target_platform']}_{marker_configuration}_{number_of_markers}pt"

    # --- 1. Define Calibration Processes ---
    # We use ExecuteProcess to run the calibration launch file as if from the terminal
    calibrate_aligner = ExecuteProcess(
        cmd=['ros2', 'launch', 'surface_tracking_aligner', 'calibration.launch.py', 'target:=aligner'],
        output='screen'
    )

    calibrate_platform = ExecuteProcess(
        cmd=['ros2', 'launch', 'surface_tracking_aligner', 'calibration.launch.py', 'target:=platform'],
        output='screen'
    )

    # --- 2. Define the Main System Nodes ---
    tools_list_str = f"['{active_camera}', '{active_target}']"
    aimooe_tracker_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(aimooe_ros2_dir, 'launch', 'aimooe_tracker.launch.py')),
        launch_arguments={'tools_to_track': tools_list_str}.items()
    )

    elfin10_l_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(elfin10_l_ros2_moveit2_dir, 'launch', 'elfin10_l.launch.py'))
    )

    elfin10_l_basic_api_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(elfin10_l_ros2_moveit2_dir, 'launch', 'elfin10_l_basic_api.launch.py'))
    )

    aligner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(aligner_dir, 'launch', 'aligner.launch.py')),
        launch_arguments={'active_camera': active_camera, 'active_target': active_target}.items()
    )

    visualizer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(visualization_dir, 'launch', 'visualizer.launch.py')),
        launch_arguments={'active_camera': active_camera, 'active_target': active_target}.items()
    )

    # Group the main system so we can launch it all at once later
    main_system_actions = [
        LogInfo(msg="=== Calibrations Complete! Launching Main System ==="),
        aimooe_tracker_launch,
        elfin10_l_sim_launch,
        elfin10_l_basic_api_launch,
        aligner_launch,
        visualizer_launch
    ]

    # --- 3. Build the Execution Chain ---
    
    # Event C: When Platform Calibration exits -> Launch the Main System
    platform_exit_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=calibrate_platform,
            on_exit=main_system_actions
        )
    )

    # Event B: When Aligner Calibration exits -> Launch Platform Calibration AND its listener
    aligner_exit_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=calibrate_aligner,
            on_exit=[
                LogInfo(msg="=== Aligner Calibrated! Starting Platform Calibration ==="),
                calibrate_platform,
                platform_exit_handler
            ]
        )
    )

    # Event A: The Initial Launch Description only starts the VERY FIRST action and its listener
    return LaunchDescription([
        LogInfo(msg="=== Starting Aligner Calibration ==="),
        calibrate_aligner,
        aligner_exit_handler
    ])