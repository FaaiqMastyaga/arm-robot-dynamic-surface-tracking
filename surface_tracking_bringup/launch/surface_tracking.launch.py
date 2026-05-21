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
    gui_dir = get_package_share_directory('surface_tracking_gui')
    estimator_dir = get_package_share_directory('surface_tracking_estimator')
    planner_dir = get_package_share_directory('surface_tracking_planner')
    controller_dir = get_package_share_directory('surface_tracking_controller')

    global_yaml_path = os.path.join(bringup_dir, 'config', 'experiment_config.yaml')

    with open(global_yaml_path, 'r') as file:
        global_config = yaml.safe_load(file)['global_experiment_manager']['ros__parameters']

    # --- Fetch Parameters ---
    use_sim = global_config.get('use_sim', True)  # Default to True if not found
    
    number_of_markers = global_config['number_of_markers']
    marker_configuration = global_config['marker_configuration']
    active_camera = global_config['active_camera_aligner']
    active_target = f"{global_config['active_target_platform']}_{marker_configuration}_{number_of_markers}pt"

    controller_type = global_config.get('controller_type', 'pid_ff')
    filter_type = global_config.get('filter_type', 'kalman_filter')

    task_frame = global_config.get('task_frame', 'whiteboard')
    base_frame = global_config.get('base_frame', 'elfin_base_link')

    # --- 1. Define Calibration Processes ---
    calibrate_aligner = ExecuteProcess(
        cmd=['ros2', 'launch', 'surface_tracking_aligner', 'calibration.launch.py', 'target:=aligner'],
        output='screen'
    )

    calibrate_platform = ExecuteProcess(
        cmd=['ros2', 'launch', 'surface_tracking_aligner', 'calibration.launch.py', 'target:=platform'],
        output='screen'
    )

    # --- 2. Define Tracking & Control Nodes ---
    tools_list_str = f"['{active_camera}', '{active_target}']"
    aimooe_tracker_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(aimooe_ros2_dir, 'launch', 'aimooe_tracker.launch.py')),
        launch_arguments={'tools_to_track': tools_list_str}.items()
    )

    aligner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(aligner_dir, 'launch', 'aligner.launch.py')),
        launch_arguments={'active_camera': active_camera, 'active_target': active_target}.items()
    )

    visualizer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(visualization_dir, 'launch', 'visualizer.launch.py')),
        launch_arguments={'active_camera': active_camera, 'active_target': active_target}.items()
    )

    dashboard_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gui_dir, 'launch', 'dashboard.launch.py')),
        launch_arguments={'use_sim_time': 'true' if use_sim else 'false'}.items()
    )

    estimator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(estimator_dir, 'launch', 'velocity_estimator.launch.py')),
        launch_arguments={'target_frame': task_frame, 'base_frame': base_frame}.items()
    )

    trajectory_generator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(planner_dir, 'launch', 'trajectory_generator.launch.py'))
    )

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(controller_dir, 'launch', 'dynamic_tracker.launch.py')),
        launch_arguments={'filter_type': filter_type, 'controller_type': controller_type}.items()
    )

    # --- 3. Conditional Robot Hardware/Sim Bringup ---
    robot_bringup_actions = []
    
    if use_sim:
        robot_bringup_actions.append(LogInfo(msg=">>> Launching Elfin SIMULATION Environment <<<"))
        robot_bringup_actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(elfin10_l_ros2_moveit2_dir, 'launch', 'elfin10_l.launch.py'))
        ))
        robot_bringup_actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(elfin10_l_ros2_moveit2_dir, 'launch', 'elfin10_l_basic_api.launch.py'))
        ))
    else:
        robot_bringup_actions.append(LogInfo(msg=">>> Launching Elfin REAL HARDWARE Environment <<<"))
        # 1. MoveGroup, RViz, and Servo (The file we edited previously)
        robot_bringup_actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(elfin10_l_ros2_moveit2_dir, 'launch', 'elfin10_l_moveit_rviz.launch.py'))
        ))
        # 2. Elfin Basic API
        robot_bringup_actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(elfin10_l_ros2_moveit2_dir, 'launch', 'elfin10_l_basic_api.launch.py'))
        ))

    # Group the main system
    main_system_actions = [
        LogInfo(msg="=== Calibrations Complete! Launching Main System ==="),
        aimooe_tracker_launch,
        aligner_launch,
        visualizer_launch,
        estimator_launch,
        controller_launch,
        trajectory_generator_launch,
        dashboard_launch
    ] + robot_bringup_actions

    # --- 4. Build the Execution Chain ---
    platform_exit_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=calibrate_platform,
            on_exit=main_system_actions
        )
    )

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

    return LaunchDescription([
        LogInfo(msg="=== Starting Aligner Calibration ==="),
        calibrate_aligner,
        aligner_exit_handler
    ])