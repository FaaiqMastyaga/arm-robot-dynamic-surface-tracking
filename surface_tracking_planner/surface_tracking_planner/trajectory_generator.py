#!/usr/bin/env python3
import time
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

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
        self.declare_parameter('lookahead_window', 10)
        self.declare_parameter('max_acceleration', 0.5)
        self.declare_parameter('enable_sharp_corner_stops', True)

        # --- Get Parameters ---
        self.control_rate = self.get_parameter('control_loop_rate').get_parameter_value().double_value
        self.drawing_feedrate = self.get_parameter('drawing_feedrate').get_parameter_value().double_value
        self.plunge_feedrate = self.get_parameter('plunge_feedrate').get_parameter_value().double_value
        self.z_hover_height = self.get_parameter('z_hover_height').get_parameter_value().double_value
        self.lookahead_window = self.get_parameter('lookahead_window').get_parameter_value().integer_value
        self.max_acceleration = self.get_parameter('max_acceleration').get_parameter_value().double_value
        self.enable_sharp_corner_stops = self.get_parameter('enable_sharp_corner_stops').get_parameter_value().bool_value

        self.dt = 1.0 / self.control_rate     # Time step for control loop
        self.tracking_active = True
        
        # --- Parameter Callback Registration ---
        self.add_on_set_parameters_callback(self.parameters_callback)

        # --- Publishers & Subscribers ---
        self.pose_pub = self.create_publisher(PoseStamped, '/desired_drawing_pose', 10)
        self.path_pub = self.create_publisher(Path, '/desired_drawing_path', 10)

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

    def parameters_callback(self, params):
        successful = True
        reason = "success"

        for param in params:
            if param.name == 'lookahead_window':
                if param.type_ == Parameter.Type.INTEGER:
                    self.lookahead_window = param.value
                    self.get_logger().info(f"Updated lookahead_window: {self.lookahead_window}")
                else:
                    successful = False
                    reason = "lookahead_window must be an integer"
            elif param.name == 'drawing_feedrate':
                if param.type_ == Parameter.Type.DOUBLE and param.value > 0.0:
                    self.drawing_feedrate = param.value
                    self.get_logger().info(f"Updated drawing_feedrate: {self.drawing_feedrate} m/s")
                else:
                    successful = False
                    reason = "drawing_feedrate must be a double"
            elif param.name == 'plunge_feedrate':
                if param.type_ == Parameter.Type.DOUBLE and param.value > 0.0:
                    self.plunge_feedrate = param.value
                    self.get_logger().info(f"Updated plunge_feedrate: {self.plunge_feedrate} m/s")
                else:
                    successful = False
                    reason = "plunge_feedrate must be a double"
            elif param.name == 'max_acceleration':
                if param.type_ == Parameter.Type.DOUBLE and param.value > 0.0:
                    self.max_acceleration = param.value
                    self.get_logger().info(f"Updated max_acceleration: {self.max_acceleration} m/s^2")
                else:
                    successful = False
                    reason = "max_acceleration must be a double strictly greater than 0.0"
            elif param.name == 'enable_sharp_corner_stops':
                if param.type_ == Parameter.Type.BOOL:
                    self.enable_sharp_corner_stops = param.value
                    state_str = "ENABLED" if self.enable_sharp_corner_stops else "DISABLED"
                    self.get_logger().info(f"Sharp Corner Exact Stops are now {state_str}")
                else:
                    successful = False
                    reason = "enable_sharp_corner_stops must be a boolean (True/False)"

        return SetParametersResult(successful=successful, reason=reason)
    
    def execute_callback(self, goal_handle):
        self.get_logger().info('--- Processing New Trajectory ---')
        raw_points = goal_handle.request.raw_pixels
        
        if len(raw_points) < 2:
            goal_handle.abort()
            return DrawTrajectory.Result(success=False, message="Not enough points drawn.")

        # --- 1. SETUP & GET CURRENT ACTUAL POSE ---
        feedback_msg = DrawTrajectory.Feedback()
        feedback_msg.current_state = "Planning Approach & Waypoints..."
        feedback_msg.progress_percentage = 0.0
        goal_handle.publish_feedback(feedback_msg)
        
        path_array = np.array([[p.x, p.y, p.z] for p in raw_points])
        smoothed_waypoints = []
        
        mpc_horizon_time = 3.0 * self.lookahead_window * self.dt
        first_point = path_array[0]
        hover_point = np.array([first_point[0], first_point[1], self.z_hover_height])

        # Get the actual current position of the pen relative to the whiteboard
        try:
            t = self.tf_buffer.lookup_transform("whiteboard", "pen_tip_link", rclpy.time.Time())
            start_pos = np.array([
                t.transform.translation.x,
                t.transform.translation.y,
                t.transform.translation.z
            ])
            self.get_logger().info("Found current robot pose. Generating approach path...")
        except Exception as e:
            self.get_logger().warn(f"TF missing for approach phase, jumping straight to hover: {e}")
            start_pos = hover_point # Fallback

        # --- 2. INTERPOLATE APPROACH (Current Pose -> Hover Point) ---
        dist_approach = np.linalg.norm(hover_point - start_pos)
        
        # We use the drawing feedrate for the transit speed so it moves predictably
        transit_speed = self.drawing_feedrate 
        
        if dist_approach > 0.001:
            steps_approach = max(int((dist_approach / transit_speed) / self.dt), 1)
            x_app = np.linspace(start_pos[0], hover_point[0], steps_approach, endpoint=False)
            y_app = np.linspace(start_pos[1], hover_point[1], steps_approach, endpoint=False)
            z_app = np.linspace(start_pos[2], hover_point[2], steps_approach, endpoint=False)
            
            for step in range(steps_approach):
                smoothed_waypoints.append([x_app[step], y_app[step], z_app[step]])
                
            # EXACT STOP at the hover point so it doesn't slam into the board diagonally
            t_stop_app = transit_speed / self.max_acceleration
            pause_steps_app = int((t_stop_app + mpc_horizon_time) / self.dt)
            for _ in range(pause_steps_app):
                smoothed_waypoints.append([hover_point[0], hover_point[1], hover_point[2]])
        else:
            smoothed_waypoints.append(hover_point)

        # --- 3. INTERPOLATE PLUNGE (Hover Point -> First Drawing Point) ---
        dist_plunge = np.linalg.norm(first_point - hover_point)
        steps_plunge = max(int((dist_plunge / self.plunge_feedrate) / self.dt), 1)
        for step in range(steps_plunge):
            z_val = np.interp(step, [0, steps_plunge], [hover_point[2], first_point[2]])
            smoothed_waypoints.append([first_point[0], first_point[1], z_val])
            
        # Add another Exact Stop at the bottom of the plunge before it starts dragging the pen!
        t_stop_plunge = self.plunge_feedrate / self.max_acceleration
        pause_steps_plunge = int((t_stop_plunge + mpc_horizon_time) / self.dt)
        for _ in range(pause_steps_plunge):
            smoothed_waypoints.append([first_point[0], first_point[1], first_point[2]])

        # --- 4. PATH SEGMENT INTERPOLATION ---
        last_corner_pos = None
        corner_window = 0.003  # 3mm window (Filters pixel jitter, preserves arcs)

        # ONE single loop for all drawing segments
        for i in range(len(path_array) - 1):
            p1 = path_array[i]
            p2 = path_array[i + 1]
            
            dist = np.linalg.norm(p2 - p1)
            if dist < 0.0001: # Skip duplicate points
                continue
                
            is_plunging = abs(p1[2] - p2[2]) > 0.001
            current_speed = self.plunge_feedrate if is_plunging else self.drawing_feedrate
            segment_time = dist / current_speed
            num_steps = max(int(segment_time / self.dt), 1)
            
            x_vals = np.linspace(p1[0], p2[0], num_steps, endpoint=False)
            y_vals = np.linspace(p1[1], p2[1], num_steps, endpoint=False)
            z_vals = np.linspace(p1[2], p2[2], num_steps, endpoint=False)
            
            for step in range(num_steps):
                smoothed_waypoints.append([x_vals[step], y_vals[step], z_vals[step]])

            # --- KINEMATIC EXACT STOP LOGIC ---
            if self.enable_sharp_corner_stops:
                check_corner = True
                
                # Lockout: Don't check for corners if we just stopped less than 10mm ago
                if last_corner_pos is not None:
                    if np.linalg.norm(p2 - last_corner_pos) < 0.01:
                        check_corner = False

                if check_corner and (i < len(path_array) - 2):
                    
                    v_in = None
                    for j in range(i, -1, -1):
                        if np.linalg.norm(p2 - path_array[j]) > corner_window:
                            v_in = (p2 - path_array[j])
                            v_in = v_in / np.linalg.norm(v_in)
                            break

                    v_out = None
                    for j in range(i + 1, len(path_array)):
                        if np.linalg.norm(path_array[j] - p2) > corner_window:
                            v_out = (path_array[j] - p2)
                            v_out = v_out / np.linalg.norm(v_out)
                            break

                    if v_in is not None and v_out is not None:
                        dot_product = np.clip(np.dot(v_in, v_out), -1.0, 1.0)
                        angle_rad = math.acos(dot_product)
                        
                        # 1.22 rad = ~70 degrees. Only stop for truly sharp vertices!
                        if angle_rad > 1.22:
                            t_stop = current_speed / self.max_acceleration
                            total_pause_time = t_stop + mpc_horizon_time
                            pause_steps = int(total_pause_time / self.dt)
                            
                            for _ in range(pause_steps):
                                smoothed_waypoints.append([p2[0], p2[1], p2[2]])
                                
                            # Update lockout position
                            last_corner_pos = p2
                
        # Ensure the very last point is added exactly once
        smoothed_waypoints.append(path_array[-1])
        total_waypoints = len(smoothed_waypoints)
        
        self.get_logger().info(f"Interpolation complete. Generated {total_waypoints} high-fidelity waypoints.")

        # --- 5. STREAMING PHASE ---
        feedback_msg.current_state = "Streaming to Robot..."
        goal_handle.publish_feedback(feedback_msg)
        
        # Stream the window path exactly at the control rate
        for idx in range(total_waypoints):
            # INSTANT ABORT CHECK: If user hit Stop or changed tabs
            if not self.tracking_active or goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().warn('Trajectory aborted by operator!')
                return DrawTrajectory.Result(success=False, message="Aborted by operator.")

            # Slice the lookahead window
            end_idx = min(idx + self.lookahead_window, total_waypoints)
            window_points = smoothed_waypoints[idx:end_idx]

            # Pad the end of the array if we are near the end to maintain a consistent window size
            while len(window_points) < self.lookahead_window:
                window_points.append(smoothed_waypoints[-1])

            # Prepare the Path message
            path_msg = Path()
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.header.frame_id = "whiteboard"

            target_roll = 0.0  # Pen points straight down
            target_pitch = 0.0
            target_yaw = 0.0 # Fallback

            try:
                t = self.tf_buffer.lookup_transform("whiteboard", "pen_tip_link", rclpy.time.Time())
                _, _, current_yaw = euler_from_quaternion(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w)
                target_yaw = current_yaw  # Keep the pen orientation aligned with the robot's current facing direction
            except Exception:
                pass

            q = quaternion_from_euler(target_roll, target_pitch, target_yaw)
            
            # Build the PoseStamped array
            for wp in window_points:
                pose = PoseStamped()
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.header.frame_id = "whiteboard"

                pose.pose.position.x = float(wp[0])
                pose.pose.position.y = float(wp[1])
                pose.pose.position.z = float(wp[2])

                pose.pose.orientation.x = float(q[0])
                pose.pose.orientation.y = float(q[1])
                pose.pose.orientation.z = float(q[2])
                pose.pose.orientation.w = float(q[3])

                path_msg.poses.append(pose)

            # Publish the full trajectory segment (for MPC)
            self.path_pub.publish(path_msg)

            # Publish the single current point (for PID)
            single_pose_msg = path_msg.poses[0]
            self.pose_pub.publish(single_pose_msg)

            if idx % 10 == 0:
                feedback_msg.progress_percentage = (idx / total_waypoints) * 100.0
                goal_handle.publish_feedback(feedback_msg)

            # Sleep to maintain the deterministic control loop
            time.sleep(self.dt) 

        # --- 6. COMPLETION PHASE ---
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