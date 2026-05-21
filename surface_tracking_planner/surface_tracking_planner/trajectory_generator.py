#!/usr/bin/env python3
import time
import math
import numpy as np
from scipy.interpolate import interp1d

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped

from surface_tracking_interfaces.action import DrawTrajectory
from tf2_ros import Buffer, TransformListener

def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return roll_x, pitch_y, yaw_z

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0; aj /= 2.0; ak /= 2.0
    ci = math.cos(ai); si = math.sin(ai)
    cj = math.cos(aj); sj = math.sin(aj)
    ck = math.cos(ak); sk = math.sin(ak)
    cc = ci*ck; cs = ci*sk; sc = si*ck; ss = si*sk
    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss
    return q

class TrajectoryGenerator(Node):
    def __init__(self):
        super().__init__('trajectory_backend_server')

        # --- Declare Parameters ---
        self.declare_parameter('control_loop_rate', 100.0)
        self.declare_parameter('drawing_feedrate', 0.1)
        self.declare_parameter('plunge_feedrate', 0.02)
        self.declare_parameter('z_hover_height', 0.01)

        # --- Get Parameters ---
        self.control_rate = self.get_parameter('control_loop_rate').get_parameter_value().double_value
        self.drawing_feedrate = self.get_parameter('drawing_feedrate').get_parameter_value().double_value
        self.plunge_feedrate = self.get_parameter('plunge_feedrate').get_parameter_value().double_value
        self.z_hover_height = self.get_parameter('z_hover_height').get_parameter_value().double_value

        self.dt = 1.0 / self.control_rate     # Time step for control loop
        self.tracking_active = True
        
        self.pose_pub = self.create_publisher(PoseStamped, '/desired_drawing_pose', 10)

        self.create_subscription(Bool, '/tracking_active_flag', lambda msg: setattr(self, 'tracking_active', msg.data), 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
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
        
        first_point = path_array[0]
        hover_point = np.array([first_point[0], first_point[1], self.z_hover_height])

        smoothed_waypoints.append(hover_point)  # Start at hover height

        dist_plunge = np.linalg.norm(first_point - hover_point)
        steps_plunge = max(int((dist_plunge / self.plunge_feedrate) / self.dt), 1)
        for step in range(steps_plunge):
            z_val = np.interp(step, [0, steps_plunge], [hover_point[2], first_point[2]])
            smoothed_waypoints.append([first_point[0], first_point[1], z_val])
            
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
            current_speed = self.plunge_feedrate if is_plunging else self.drawing_feedrate
            
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
            # INSTANT ABORT CHECK: If user hit Stop or changed tabs
            if not self.tracking_active or goal_handle.is_cancel_requested:
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

            target_roll = math.pi  # Pen points straight down
            target_pitch = 0.0
            target_yaw = 0.0 # Fallback

            try:
                t = self.tf_buffer.lookup_transform("whiteboard", "pen_tip_link", rclpy.time.Time())
                _, _, current_yaw = euler_from_quaternion(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w)
                target_yaw = current_yaw  # Keep the pen orientation aligned with the robot's current facing direction
            except Exception:
                pass

            q = quaternion_from_euler(target_roll, target_pitch, target_yaw)
            msg.pose.orientation.x = float(q[0])
            msg.pose.orientation.y = float(q[1])
            msg.pose.orientation.z = float(q[2])
            msg.pose.orientation.w = float(q[3])

            # Publish to the C++ tracking loop
            self.pose_pub.publish(msg)

            # Send live progress bar updates to the GUI
            if idx % 10 == 0: # Throttle feedback to avoid network spam
                feedback_msg.progress_percentage = (idx / total_waypoints) * 100.0
                goal_handle.publish_feedback(feedback_msg)

            # Sleep to maintain the deterministic control loop
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