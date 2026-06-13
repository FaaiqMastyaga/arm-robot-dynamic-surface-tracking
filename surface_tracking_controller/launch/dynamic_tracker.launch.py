import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression, Command, FindExecutable
from launch.conditions import IfCondition
from launch_ros.actions import Node

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:
        return None

def generate_launch_description():
    global_config_file = os.path.join(
        get_package_share_directory('surface_tracking_bringup'),
        'config',
        'experiment_config.yaml'
    )

    config_file = os.path.join(
        get_package_share_directory('surface_tracking_controller'),
        'config',
        'controller_config.yaml'
    )

    # --- Fetch Parameters ---
    with open(global_config_file, 'r') as file:
        global_config = yaml.safe_load(file)['global_experiment_manager']['ros__parameters']
    
    z_plunge_depth = global_config['z_plunge_depth']

    # --- Launch Arguments ---
    filter_type_arg = DeclareLaunchArgument(
        'filter_type',
        default_value='kalman_filter',
        choices=['raw', 'ema_filter', 'kalman_filter'],
        description='Filtering method for velocity estimation'
    )

    controller_type_arg = DeclareLaunchArgument(
        'controller_type',
        default_value='mpc',
        choices=['pid', 'pid_ff', 'mpc'],
        description='Type of controller to use'
    )

    # --- Parse Robot Description (URDF) ---
    robot_description_config = Command([
        FindExecutable(name='xacro'), ' ', 
        os.path.join(get_package_share_directory("elfin10_l_ros2_gazebo"), "urdf", "elfin10_l.urdf.xacro"),
        ' use_fake_hardware:=false',
        ' use_real_hardware:=true'
    ])
    robot_description = {'robot_description': robot_description_config}

    # --- Parse Semantic Description (SRDF) ---
    robot_description_semantic_config = load_file(
        "elfin10_l_ros2_moveit2", "config/elfin10_l.srdf"
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    # --- Conditional Logic ---
    is_mpc = PythonExpression(["'", LaunchConfiguration('controller_type'), "' == 'mpc'"])
    is_not_mpc = PythonExpression(["'", LaunchConfiguration('controller_type'), "' != 'mpc'"])

    # --- Node Definitions ---
    # 1. The PID/PID_FF Node
    pid_node = Node(
        package='surface_tracking_controller',
        executable='dynamic_tracker',
        name='dynamic_tracking',
        output='screen',
        condition=IfCondition(is_not_mpc),
        remappings=[
            ('/target_twist', ['/estimated_target_twist/', LaunchConfiguration('filter_type')]),
        ],
        parameters=[
            config_file,
            {'controller_type': LaunchConfiguration('controller_type')}
        ]
    )

    # 2. The LTV-MPC Node
    mpc_node = Node(
        package='surface_tracking_controller',
        executable='mpc_tracker',
        name='mpc_tracking',
        output='screen',
        condition=IfCondition(is_mpc),
        remappings=[
            ('/target_twist', ['/estimated_target_twist/', LaunchConfiguration('filter_type')]),
        ],
        parameters=[
            config_file,
            {'z_plunge_depth': float(z_plunge_depth)},
            robot_description,          # Inject URDF directly
            robot_description_semantic  # Inject SRDF directly
        ]
    )

    return LaunchDescription([
        filter_type_arg,
        controller_type_arg,
        pid_node,
        mpc_node
    ])