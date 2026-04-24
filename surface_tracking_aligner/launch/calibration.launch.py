import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('surface_tracking_aligner'),
        'config',
        'marker_config.yaml'
    )

    calibration_node = Node(
        package='surface_tracking_aligner',
        executable='calibration.py',
        name='calibration',
        output='screen',
        parameters=[{
            'yaml_path': '/home/dian/faaiq/Tugas_Akhir/dynamic_surface_tracking_ws/src/dynamic_surface_tracking/surface_tracking_aligner/config/marker_config.yaml',
            'target_node_name': 'camera_aligner',
            'aimtool_path': '/home/dian/.aimooe/tools/surface_tracking/robot_base.aimtool'
        }]
    )

    return LaunchDescription([
        calibration_node,
    ])