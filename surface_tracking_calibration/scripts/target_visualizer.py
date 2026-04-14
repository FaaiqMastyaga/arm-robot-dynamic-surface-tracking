#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Time
import numpy as np

class TargetVisualizer(Node):
    def __init__(self):
        super().__init__('target_visualizer')
        
        # Publishers
        self.mesh_pub = self.create_publisher(Marker, 'target_mesh_marker', 10)
        self.spheres_pub = self.create_publisher(MarkerArray, 'target_spheres', 10)
        self.board_pub = self.create_publisher(Marker, 'whiteboard_marker', 10)
        
        # --- Read ROS2 Parameters ---
        self.declare_parameter('cad_points', [0.0] * 12)

        # Fetch Parameter from YAML
        self.target_cad_points = np.array(self.get_parameter('cad_points').get_parameter_value().double_array_value).reshape(-1, 3)

        self.timer = self.create_timer(0.1, self.publish_markers)
        self.get_logger().info("Target Mesh and Spheres visualizer started.")

    def publish_markers(self):
        now = Time()

        # ---------------------------------------------------------
        # 1. PUBLISH THE 4 TRACKING SPHERES
        # ---------------------------------------------------------
        sphere_array = MarkerArray()
        for i, pt in enumerate(self.target_cad_points):
            sphere = Marker()
            sphere.header.frame_id = "target_platform_aligned"
            sphere.header.stamp = now
            sphere.ns = "target_spheres"
            sphere.id = i
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD

            # 10mm spheres (0.01 meters)
            sphere.scale.x = 0.01
            sphere.scale.y = 0.01
            sphere.scale.z = 0.01

            sphere.pose.position.x = pt[0]
            sphere.pose.position.y = pt[1]
            sphere.pose.position.z = pt[2]
            sphere.pose.orientation.w = 1.0

            # Color: Grey
            sphere.color.r = 0.64
            sphere.color.g = 0.64
            sphere.color.b = 0.64
            sphere.color.a = 1.0

            sphere_array.markers.append(sphere)

        self.spheres_pub.publish(sphere_array)

        # ---------------------------------------------------------
        # 2. PUBLISH THE STL MESH
        # ---------------------------------------------------------
        mesh_marker = Marker()
        mesh_marker.header.frame_id = "target_platform_aligned"
        mesh_marker.header.stamp = now
        mesh_marker.ns = "target_mesh"
        mesh_marker.id = 0
        
        # Tell RViz to load a 3D file!
        mesh_marker.type = Marker.MESH_RESOURCE
        mesh_marker.action = Marker.ADD

        # The path to STL inside the ROS 2 package
        mesh_marker.mesh_resource = "package://surface_tracking_calibration/meshes/target_platform_frame.stl"

        # to meters (multiply by 0.001) for RViz!
        mesh_marker.scale.x = 0.001
        mesh_marker.scale.y = 0.001
        mesh_marker.scale.z = 0.001

        # Keep origin at 0,0,0 assuming Onshape origin matches CAD points
        mesh_marker.pose.position.x = 0.0
        mesh_marker.pose.position.y = 0.0
        mesh_marker.pose.position.z = 0.0

        mesh_marker.pose.orientation.x = 0.0
        mesh_marker.pose.orientation.y = 0.0
        mesh_marker.pose.orientation.z = 0.0
        mesh_marker.pose.orientation.w = 1.0

        # Color: Blue
        mesh_marker.color.r = 0.0
        mesh_marker.color.g = 0.01
        mesh_marker.color.b = 0.2
        mesh_marker.color.a = 1.0

        self.mesh_pub.publish(mesh_marker)

        # ---------------------------------------------------------
        # 3. PUBLISH THE WHITEBOARD
        # ---------------------------------------------------------
        board = Marker()
        # Anchor strictly to the new TF frame!
        board.header.frame_id = "whiteboard"
        board.header.stamp = now
        board.ns = "whiteboard"
        board.id = 1 # Must be different from the mesh ID
        board.type = Marker.CUBE
        board.action = Marker.ADD

        # Dimensions: 30cm x 20cm x 1.3cm thick
        board.scale.x = 0.30
        board.scale.y = 0.20
        board.scale.z = 0.013

        # Offset
        board.pose.position.x = 0.0
        board.pose.position.y = 0.0
        board.pose.position.z = -0.013 / 2.0  # Adjusted so top surface is at z=0 in the whiteboard frame
        board.pose.orientation.x = 0.0
        board.pose.orientation.y = 0.0
        board.pose.orientation.z = 0.0
        board.pose.orientation.w = 1.0

        # Appearance: white
        board.color.r = 1.0
        board.color.g = 1.0
        board.color.b = 1.0
        board.color.a = 1.0 

        self.board_pub.publish(board)

def main(args=None):
    rclpy.init(args=args)
    node = TargetVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()