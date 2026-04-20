#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
from scipy.spatial.transform import Rotation as R
import itertools
from aimooe_msgs.msg import ToolArray

class TargetAligner(Node):
    def __init__(self):
        super().__init__('target_aligner')
        
        # --- Read ROS2 Parameters ---
        self.declare_parameter('tool_name', 'target_platform')
        self.declare_parameter('cad_points', [0.0] * 12)

        # Fetch Parameter from YAML
        self.tool_name = self.get_parameter('tool_name').get_parameter_value().string_value
        self.cad_points = np.array(self.get_parameter('cad_points').get_parameter_value().double_array_value).reshape(-1, 3)

        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.subscription = self.create_subscription(
            ToolArray, 
            '/aimooe/tool_info', 
            self.tool_callback, 
            10
        )
            
        self.get_logger().info("Target Aligner Node Started.")

    def calculate_transform(self, cad_pts, cam_pts):
        centroid_cad = np.mean(cad_pts, axis=0)
        centroid_cam = np.mean(cam_pts, axis=0)
        centered_cad = cad_pts - centroid_cad
        centered_cam = cam_pts - centroid_cam
        rotation, rms_error = R.align_vectors(centered_cam, centered_cad)
        translation = centroid_cam - rotation.apply(centroid_cad)
        return rotation, translation, rms_error

    def auto_register(self, valid_camera_points):
        """ Tests all 24 permutations to find the lowest RMS error """
        min_error = float('inf')
        best_rot = None
        best_trans = None

        num_pts = len(valid_camera_points)

        cad_combinations = list(itertools.permutations(self.cad_points, num_pts))

        for cad_subset in cad_combinations:
            cad_subset_array = np.array(cad_subset)

            for perm in itertools.permutations(valid_camera_points):
                perm_array = np.array(perm)
                rot, trans, error = self.calculate_transform(cad_subset_array, perm_array)

                if error < min_error:
                    min_error = error
                    best_rot = rot
                    best_trans = trans

        return best_rot, best_trans, min_error

    def tool_callback(self, msg):
        for tool in msg.tools:
            if tool.tool_name == self.tool_name:
                
                # Filter out dropped (0,0,0) markers
                valid_camera_points = []
                for pt in tool.marker_points:
                    if not (pt.x == 0.0 and pt.y == 0.0 and pt.z == 0.0):
                        valid_camera_points.append([pt.x/1000.0, pt.y/1000.0, pt.z/1000.0])  # Convert mm to meters

                # Only proceed if we have the minimum 3 points required for 6-DOF
                if len(valid_camera_points) >= 3:
                    valid_camera_points = np.array(valid_camera_points)
                    
                    # Run SVD Permutation
                    rot, trans, error = self.auto_register(valid_camera_points)

                if error < 0.02: 
                    t = TransformStamped()
                    t.header.stamp = msg.header.stamp
                    t.header.frame_id = 'aimooe_camera_link'
                    
                    # Publish a mathematically perfect frame!
                    t.child_frame_id = 'target_platform_aligned'

                    # NO INVERSION NEEDED! 
                    t.transform.translation.x = trans[0]
                    t.transform.translation.y = trans[1]
                    t.transform.translation.z = trans[2]

                    quat = rot.as_quat() 
                    t.transform.rotation.x = quat[0]
                    t.transform.rotation.y = quat[1]
                    t.transform.rotation.z = quat[2]
                    t.transform.rotation.w = quat[3]

                    self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = TargetAligner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()