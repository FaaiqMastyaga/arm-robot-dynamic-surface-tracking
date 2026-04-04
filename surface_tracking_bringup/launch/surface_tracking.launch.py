import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    aimooe_ros2_dir = get_package_share_directory('aimooe_ros2')
    elfin10_l_ros2_moveit2_dir = get_package_share_directory('elfin10_l_ros2_moveit2')

    # Start the Physical Optical Tracker Sensor
    aimooe_tracker_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(aimooe_ros2_dir, 'launch', 'aimooe_tracker.launch.py')
        )
    )

    # Start the Simulation
    elfin10_l_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(elfin10_l_ros2_moveit2_dir, 'launch', 'elfin10_l.launch.py')
        )
    )

    # Start the Robot API
    elfin10_l_basic_api_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(elfin10_l_ros2_moveit2_dir, 'launch', 'elfin10_l_basic_api.launch.py')
        )
    )

    camera_aligner_node = Node(
        package='surface_tracking_calibration',
        executable='camera_aligner.py',
        name='camera_aligner'
    )

    return LaunchDescription([
        aimooe_tracker_launch,
        elfin10_l_sim_launch,
        elfin10_l_basic_api_launch,
        camera_aligner_node,
    ])