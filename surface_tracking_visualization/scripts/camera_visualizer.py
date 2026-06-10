#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Time
import math

class CameraVisualizer(Node):
    def __init__(self):
        super().__init__('camera_visualizer')
        
        self.publisher_ = self.create_publisher(Marker, 'camera_mesh_marker', 10)
        
        self.timer = self.create_timer(0.01, self.publish_marker)
        
        self.get_logger().info("Aimooe Optical Tracker visualizer started.")

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

    def publish_marker(self):
        marker = Marker()
        # Anchor strictly to the camera's TF frame
        marker.header.frame_id = "aimooe_camera_link"
        marker.header.stamp = Time()  # Zero timestamp to prevent RViz queue drops!
        marker.ns = "camera_mesh"
        marker.id = 0
        
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD

        # Path to STL
        marker.mesh_resource = "package://surface_tracking_visualization/meshes/aimooe_optical_tracker.stl"

        # Scale down from mm to meters
        marker.scale.x = 0.001
        marker.scale.y = 0.001
        marker.scale.z = 0.001

        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0

        roll = -1.57
        pitch = 0.0
        yaw = 3.14
        q = self.euler_to_quaternion(roll, pitch, yaw)
        
        marker.pose.orientation.x = q[0]
        marker.pose.orientation.y = q[1]
        marker.pose.orientation.z = q[2]
        marker.pose.orientation.w = q[3]

        # Appearance: White
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0  # Solid

        self.publisher_.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = CameraVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()