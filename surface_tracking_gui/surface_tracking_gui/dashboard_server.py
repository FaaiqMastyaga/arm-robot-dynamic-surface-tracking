#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import math

# Standard ROS 2 Messages
from geometry_msgs.msg import TwistStamped, PoseStamped
from sensor_msgs.msg import JointState
from control_msgs.msg import JointJog
from std_msgs.msg import Bool, Int8
from nav_msgs.msg import Path
from std_srvs.srv import Trigger

# TF2
from tf2_ros import Buffer, TransformListener

# Custom Interfaces
from surface_tracking_interfaces.msg import DashboardStatus, JogCommand

class DashboardServer(Node):
    def __init__(self):
        super().__init__('dashboard_server')

        # --- Frame Definitions ---
        self.base_frame = 'elfin_base_link'
        self.tip_frame = 'pen_tip_link'

        # --- Internal State ---
        self.status_msg = DashboardStatus()
        self.executed_path = Path()
        self.executed_path.header.frame_id = self.base_frame
        self.is_tracking_active = False

        # --- TF2 Setup ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ==========================================
        # PUBLISHERS
        # ==========================================
        # 1. State Aggregator (To GUI)
        self.status_pub = self.create_publisher(DashboardStatus, '/dashboard/status', 10)
        
        # 2. RViz Visuals
        self.path_pub = self.create_publisher(Path, '/executed_trajectory', 10)
        
        # 3. MoveIt Servo Commands
        self.servo_twist_pub = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.servo_joint_pub = self.create_publisher(JointJog, '/servo_node/delta_joint_cmds', 10)

        # ==========================================
        # SUBSCRIBERS
        # ==========================================
        # 1. Hardware/System States
        self.create_subscription(Bool, '/enable_state', self.cb_enable, 10)
        self.create_subscription(Bool, '/fault_state', self.cb_fault, 10)
        self.create_subscription(Int8, '/servo_node/status', self.cb_servo_status, 10)
        self.create_subscription(JointState, '/joint_states', self.cb_joint_states, 10)

        # 2. GUI Commands
        self.create_subscription(JogCommand, '/dashboard/jog_cmds', self.cb_jog_commands, 10)

        # 3. Multiplexer
        self.create_subscription(Bool, '/tracking_active_flag', self.cb_tracking_flag, 10)
        self.create_subscription(TwistStamped, '/tracking/delta_twist_cmds', self.cb_tracking_twist, 10)

        # ==========================================
        # SERVICES PROVIDED
        # ==========================================
        # Allow GUI to clear the RViz line
        self.create_service(Trigger, '/dashboard/clear_rviz_path', self.srv_clear_path)

        # ==========================================
        # HIGH FREQUENCY LOOP (20 Hz)
        # ==========================================
        self.timer = self.create_timer(0.05, self.high_frequency_loop)
        
        self.get_logger().info("Dashboard Server is running. Awaiting GUI connections...")

    # --- Hardware Callbacks ---
    def cb_enable(self, msg: Bool): 
        self.status_msg.is_enabled = msg.data

    def cb_fault(self, msg: Bool): 
        self.status_msg.is_faulted = msg.data

    def cb_servo_status(self, msg: Int8): 
        self.status_msg.servo_status = msg.data

    def cb_joint_states(self, msg: JointState):
        target_joints = ['elfin_joint1', 'elfin_joint2', 'elfin_joint3', 'elfin_joint4', 'elfin_joint5', 'elfin_joint6']
        try:
            for i, target_name in enumerate(target_joints):
                if target_name in msg.name:
                    idx = msg.name.index(target_name)
                    self.status_msg.joint_positions[i] = math.degrees(msg.position[idx])
        except Exception:
            pass

    # --- Multiplexer Logic
    def cb_tracking_flag(self, msg: Bool):
        if self.is_tracking_active and not msg.data:
            twist = TwistStamped()
            twist.header.stamp = self.get_clock().now().to_msg()
            twist.header.frame_id = self.base_frame
            self.servo_twist_pub.publish(twist)
            self.get_logger().info('Tracking disabled. Auto-brake applied to robot.')

        self.is_tracking_active = msg.data

    def cb_tracking_twist(self, msg: TwistStamped):
        # Only allow controller node to move the robot if tracking is active
        if self.is_tracking_active:
            self.servo_twist_pub.publish(msg)

    def cb_jog_commands(self, msg: JogCommand):
        # Block jogging if tracking is currently active to prevent collisions
        if self.is_tracking_active:
            self.get_logger().warn('Jog blocked: Canvas Tracking is active!', throttle_duration_sec=2.0)
            return

        if msg.jog_type == 'cart':
            twist = TwistStamped()
            twist.header.stamp = self.get_clock().now().to_msg()
            twist.header.frame_id = msg.frame_id
            
            val = msg.direction * msg.velocity_scale
            if msg.axis == 0: twist.twist.linear.x = val
            elif msg.axis == 1: twist.twist.linear.y = val
            elif msg.axis == 2: twist.twist.linear.z = val
            elif msg.axis == 3: twist.twist.angular.x = val
            elif msg.axis == 4: twist.twist.angular.y = val
            elif msg.axis == 5: twist.twist.angular.z = val
            
            self.servo_twist_pub.publish(twist)

        elif msg.jog_type == 'joint':
            jog = JointJog()
            jog.header.stamp = self.get_clock().now().to_msg()
            jog.header.frame_id = self.base_frame
            jog.joint_names = [f'elfin_joint{msg.axis + 1}']
            jog.velocities = [msg.direction * msg.velocity_scale]
            
            self.servo_joint_pub.publish(jog)

    def srv_clear_path(self, request, response):
        """Clears the RViz trajectory history."""
        self.executed_path.poses.clear()
        self.executed_path.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.executed_path)
        response.success = True
        response.message = "RViz path cleared."
        return response

    # --- Core Logic Loop ---
    def high_frequency_loop(self):
        try:
            # 1. Lookup TF safely
            t_tf = self.tf_buffer.lookup_transform(self.base_frame, self.tip_frame, rclpy.time.Time())
            
            # 2. Update Status Message for GUI
            self.status_msg.x = t_tf.transform.translation.x
            self.status_msg.y = t_tf.transform.translation.y
            self.status_msg.z = t_tf.transform.translation.z
            self.status_msg.qx = t_tf.transform.rotation.x
            self.status_msg.qy = t_tf.transform.rotation.y
            self.status_msg.qz = t_tf.transform.rotation.z
            self.status_msg.qw = t_tf.transform.rotation.w
            
            # Publish aggregated state to GUI
            self.status_pub.publish(self.status_msg)

            # 3. Handle RViz Path Recording
            pose = PoseStamped()
            pose.header.stamp = t_tf.header.stamp # Sync with Gazebo/TF Time
            pose.header.frame_id = self.base_frame
            pose.pose.position.x = t_tf.transform.translation.x
            pose.pose.position.y = t_tf.transform.translation.y
            pose.pose.position.z = t_tf.transform.translation.z
            pose.pose.orientation = t_tf.transform.rotation

            # Only append if moved > 1mm
            if len(self.executed_path.poses) == 0:
                self.executed_path.poses.append(pose)
            else:
                last_pose = self.executed_path.poses[-1]
                dx = pose.pose.position.x - last_pose.pose.position.x
                dy = pose.pose.position.y - last_pose.pose.position.y
                dz = pose.pose.position.z - last_pose.pose.position.z
                if math.sqrt(dx*dx + dy*dy + dz*dz) > 0.001:
                    self.executed_path.poses.append(pose)
                    if len(self.executed_path.poses) > 10000:
                        self.executed_path.poses.pop(0)

            self.executed_path.header.stamp = t_tf.header.stamp
            self.path_pub.publish(self.executed_path)

        except Exception as e:
            # Throttle warnings so it doesn't flood the terminal if TF drops briefly
            self.get_logger().warn(f'TF Lookup missed: {e}', throttle_duration_sec=2.0)

def main(args=None):
    rclpy.init(args=args)
    node = DashboardServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()