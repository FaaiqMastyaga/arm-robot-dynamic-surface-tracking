#!/usr/bin/env python3
import time
import math
import numpy as np
from scipy.interpolate import interp1d

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import PoseStamped

from surface_tracking_interfaces.action import DrawTrajectory

class TrajectoryGenerator(Node):
    def __init__(self):
        super().__init__('trajectory_backend_server')

        # --- Declare Parameters ---
        self.declare_parameter('control_loop_rate_hz', 50.0)
        self.declare_parameter('drawing_feedrate_m_s', 0.025)
        self.declare_parameter('plunge_feedrate_m_s', 0.02)

        # --- Get Parameters ---
        self.control_rate_hz = self.get_parameter('control_loop_rate_hz').get_parameter_value().double_value
        self.drawing_feedrate_m_s = self.get_parameter('drawing_feedrate_m_s').get_parameter_value().double_value
        self.plunge_feedrate_m_s = self.get_parameter('plunge_feedrate_m_s').get_parameter_value().double_value

        self.dt = 1.0 / self.control_rate_hz     # Time step for control loop
        
        self.pose_pub = self.create_publisher(PoseStamped, '/desired_drawing_pose', 10)
        
        self._action_server = ActionServer(
            self,
            DrawTrajectory,
            'execute_drawing',
            self.execute_callback
        )
        self.get_logger().info("Trajectory Engine Online. Waiting for Canvas streams...")

    def execute_callback(self, goal_handle):
        self.get_logger().info('--- Processing New Trajectory ---')
        raw_points = goal_handle.request.raw_pixels
        
        if len(raw_points) < 2:
            goal_handle.abort()
            return DrawTrajectory.Result(success=False, message="Not enough points drawn.")

        # --- 1. INTERPOLATION PHASE ---
        feedback_msg = DrawTrajectory.Feedback()
        feedback_msg.current_state = "Interpolating Waypoints..."
        feedback_msg.progress_percentage = 0.0
        goal_handle.publish_feedback(feedback_msg)
        
        # Convert ROS Point[] to a numpy array for fast math: [[x, y, z], [x, y, z]...]
        path_array = np.array([[p.x, p.y, p.z] for p in raw_points])
        
        smoothed_waypoints = []
        
        # Iterate through the segments
        for i in range(len(path_array) - 1):
            p1 = path_array[i]
            p2 = path_array[i + 1]
            
            # Calculate Euclidean distance of this specific segment
            dist = np.linalg.norm(p2 - p1)
            if dist < 0.0001: # Skip duplicate points
                continue
                
            # Determine speed based on whether we are moving in Z (plunging) or XY (drawing)
            is_plunging = abs(p1[2] - p2[2]) > 0.001
            current_speed = self.plunge_feedrate_m_s if is_plunging else self.drawing_feedrate_m_s
            
            # Calculate how much time this segment should take
            segment_time = dist / current_speed
            
            # Calculate how many 50Hz control loops fit into that time
            num_steps = max(int(segment_time / self.dt), 1)
            
            # Linearly interpolate between the two points to create perfectly spaced waypoints
            x_vals = np.linspace(p1[0], p2[0], num_steps, endpoint=False)
            y_vals = np.linspace(p1[1], p2[1], num_steps, endpoint=False)
            z_vals = np.linspace(p1[2], p2[2], num_steps, endpoint=False)
            
            for step in range(num_steps):
                smoothed_waypoints.append([x_vals[step], y_vals[step], z_vals[step]])
                
        # Ensure the very last point is added
        smoothed_waypoints.append(path_array[-1])
        total_waypoints = len(smoothed_waypoints)
        
        self.get_logger().info(f"Interpolation complete. Generated {total_waypoints} high-fidelity waypoints.")

        # --- 2. STREAMING PHASE ---
        feedback_msg.current_state = "Streaming to Robot..."
        goal_handle.publish_feedback(feedback_msg)
        
        # Stream the points exactly at the control rate
        for idx, wp in enumerate(smoothed_waypoints):
            # Safety Check: Did the user hit E-Stop or Cancel on the GUI?
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().warn('Trajectory aborted by operator!')
                return DrawTrajectory.Result(success=False, message="Aborted by operator.")

            # Construct the pose
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "whiteboard"
            
            msg.pose.position.x = float(wp[0])
            msg.pose.position.y = float(wp[1])
            msg.pose.position.z = float(wp[2])
            
            # Pen points straight down (assuming the board Z points UP)
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 1.0  # Adjust quaternion based on your specific end-effector tool orientation
            msg.pose.orientation.z = 0.0
            msg.pose.orientation.w = 0.0

            # Publish to the C++ tracking loop
            self.pose_pub.publish(msg)

            # Send live progress bar updates to the GUI
            if idx % 10 == 0: # Throttle feedback to avoid network spam
                feedback_msg.progress_percentage = (idx / total_waypoints) * 100.0
                goal_handle.publish_feedback(feedback_msg)

            # Sleep to maintain the deterministic 50Hz control loop
            time.sleep(self.dt) 

        # --- 3. COMPLETION PHASE ---
        goal_handle.succeed()
        
        result = DrawTrajectory.Result()
        result.success = True
        result.message = "Trajectory execution completed smoothly."
        self.get_logger().info('--- Trajectory Complete ---')
        
        return result

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryGenerator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()