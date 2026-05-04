#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import yaml
import math
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped, Point
from sensor_msgs.msg import JointState
from control_msgs.msg import JointJog
from std_msgs.msg import Bool, Float32, Int8
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster
from rclpy.action import ActionClient

# Custom Action Interface
from surface_tracking_interfaces.action import DrawTrajectory

# PySide6 Imports
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                               QHBoxLayout, QGridLayout, QLabel, QLineEdit, 
                               QPushButton, QGroupBox, QRadioButton, 
                               QButtonGroup, QTabWidget, QSlider, QProgressBar,
                               QSizePolicy)
from PySide6.QtGui import QPainter, QPen, QColor
from PySide6.QtCore import QThread, Signal, Slot, Qt, QTimer, QRectF, QPointF

# ---------------------------------------------------------
# 1. THE ROS WORKER THREAD (Backend Communication)
# ---------------------------------------------------------
class RosWorker(QThread):
    # Old Signals
    ee_pose_signal = Signal(float, float, float, float, float, float, float)
    service_response_signal = Signal(bool, str)
    servo_state_signal = Signal(bool)
    fault_state_signal = Signal(bool)
    joint_state_signal = Signal(list)
    servo_status_signal = Signal(int)
    tracking_error_signal = Signal(float, float, float, float)

    # New Action Signals
    action_feedback_signal = Signal(float, str)
    action_result_signal = Signal(bool, str)

    def __init__(self):
        super().__init__()
        self.node = None

        self.base_frame = 'elfin_base_link'
        self.ee_frame = 'elfin_end_link'
        self.current_ee_display_frame = self.base_frame
        
        self.current_vel_scale = 0.4
        self.tracking_enabled = False

    def run(self):
        rclpy.init(args=sys.argv)
        self.node = Node('elfin_master_dashboard')

        # --- Publishers ---
        self.cart_publisher_ = self.node.create_publisher(PoseStamped, '/cart_goal', 10)
        self.jog_twist_pub_ = self.node.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.jog_joint_pub_ = self.node.create_publisher(JointJog, '/servo_node/delta_joint_cmds', 10)
        self.joint_publisher_ = self.node.create_publisher(JointState, '/joint_goal', 10)
        self.vel_publisher_ = self.node.create_publisher(Float32, '/vel', 10)
        self.tracking_mode_pub_ = self.node.create_publisher(Bool, '/tracking_active_flag', 10) # Custom flag for your backend

        # --- Service Clients ---
        self.client_map = {
            'enable': self.node.create_client(SetBool, '/enable_robot'),
            'disable': self.node.create_client(SetBool, '/disable_robot'),
            'clear': self.node.create_client(SetBool, '/clear_fault'),
            'home': self.node.create_client(SetBool, '/home_teleop'),
            'stop': self.node.create_client(SetBool, '/stop_teleop'),
            'start_servo': self.node.create_client(Trigger, '/servo_node/start_servo'),
            'stop_servo': self.node.create_client(Trigger, '/servo_node/stop_servo')
        }

        # --- Action Client ---
        self.draw_client = ActionClient(self.node, DrawTrajectory, 'execute_drawing')

        # --- Subscribers ---
        self.node.create_subscription(Bool, '/enable_state', self.cb_enable, 10)
        self.node.create_subscription(Bool, '/fault_state', self.cb_fault, 10)
        self.node.create_subscription(JointState, '/joint_states', self.cb_joint_states, 10)
        self.node.create_subscription(Int8, '/servo_node/status', self.cb_servo_status, 10)

        # --- TF & Timers ---
        self.tf_buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_, self.node)
        self.tf_static_broadcaster_ = StaticTransformBroadcaster(self.node)
        self.node.create_timer(0.2, self.periodic_tf_lookup)

        rclpy.spin(self.node)
        self.node.destroy_node()
        rclpy.shutdown()

    # --- Callbacks ---
    def cb_enable(self, msg): self.servo_state_signal.emit(msg.data)
    def cb_fault(self, msg): self.fault_state_signal.emit(msg.data)
    def cb_servo_status(self, msg): self.servo_status_signal.emit(msg.data)
    def cb_joint_states(self, msg):
        target_joints = ['elfin_joint1', 'elfin_joint2', 'elfin_joint3', 'elfin_joint4', 'elfin_joint5', 'elfin_joint6']
        current_vals = [0.0] * 6
        try:
            for i, target_name in enumerate(target_joints):
                if target_name in msg.name:
                    idx = msg.name.index(target_name)
                    current_vals[i] = math.degrees(msg.position[idx])
            self.joint_state_signal.emit(current_vals)
        except Exception: pass

    def periodic_tf_lookup(self):
        try:
            t = self.tf_buffer_.lookup_transform(self.current_ee_display_frame, self.ee_frame, rclpy.time.Time())
            p = t.transform.translation; r = t.transform.rotation
            self.ee_pose_signal.emit(p.x, p.y, p.z, r.x, r.y, r.z, r.w)
        except Exception: pass

    # --- Actions & Services ---
    def call_service(self, action_name):
        client = self.client_map.get(action_name)
        if not client: 
            self.service_response_signal.emit(False, f"Client {action_name} not defined")
            return
        if client.wait_for_service(timeout_sec=1.0):
            req = SetBool.Request() if client.srv_type == SetBool else Trigger.Request()
            if hasattr(req, 'data'): req.data = True
            future = client.call_async(req)
            future.add_done_callback(lambda f: self.service_response_signal.emit(f.result().success, f.result().message))
        else:
            self.service_response_signal.emit(False, f"Service {action_name} not available")

    def set_tracking_ready_mode(self, active):
        # Publish a boolean flag to tell your backend C++ code to hover over the whiteboard
        msg = Bool()
        msg.data = active
        self.tracking_mode_pub_.publish(msg)
        self.service_response_signal.emit(True, f"Tracking Ready Mode: {'ON' if active else 'OFF'}")

    def send_drawing_goal(self, points_3d, scale_factor):
        if not self.draw_client.wait_for_server(timeout_sec=1.0):
            self.action_result_signal.emit(False, "Backend Action Server Offline!")
            return

        goal_msg = DrawTrajectory.Goal()
        goal_msg.canvas_scale_factor = scale_factor
        for pt in points_3d:
            p = Point()
            p.x, p.y, p.z = float(pt[0]), float(pt[1]), float(pt[2])
            goal_msg.raw_pixels.append(p)

        self.send_goal_future = self.draw_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.action_feedback_signal.emit(fb.progress_percentage, fb.current_state)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.action_result_signal.emit(False, "Robot rejected trajectory.")
            return
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        res = future.result().result
        self.action_result_signal.emit(res.success, res.message)

    # --- Publishing Helpers ---
    def publish_goal(self, pos, orient, frame_id):
        q = np.array(orient)
        norm = np.linalg.norm(q)
        q = q / norm if norm != 0 else [0,0,0,1]
        goal = PoseStamped()
        goal.header.frame_id = frame_id
        goal.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = pos
        goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w = q
        self.cart_publisher_.publish(goal)

    def publish_joint_goal(self, degrees_list):
        rads = [math.radians(d) for d in degrees_list]
        msg = JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.name = [f'elfin_joint{i+1}' for i in range(6)]
        msg.position = rads
        self.joint_publisher_.publish(msg)

    def publish_velocity_scale(self, scale_percent):
        msg = Float32()
        msg.data = float(scale_percent / 100.0)
        self.vel_publisher_.publish(msg)

    def publish_static_tf(self, pos, orient):
        t = TransformStamped()
        t.header.stamp = self.node.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'elfin_base_link'
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = pos
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = orient
        self.tf_static_broadcaster_.sendTransform(t)

    def send_cartesian_jog(self, axis_index, value, frame_id):
        msg = TwistStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        if axis_index == 0: msg.twist.linear.x = value
        elif axis_index == 1: msg.twist.linear.y = value
        elif axis_index == 2: msg.twist.linear.z = value
        elif axis_index == 3: msg.twist.angular.x = value
        elif axis_index == 4: msg.twist.angular.y = value
        elif axis_index == 5: msg.twist.angular.z = value
        self.jog_twist_pub_.publish(msg)

    def send_joint_jog(self, joint_index, value):
        msg = JointJog()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "elfin_base_link"
        msg.joint_names = [f'elfin_joint{joint_index+1}']
        msg.velocities = [value]
        self.jog_joint_pub_.publish(msg)

# ---------------------------------------------------------
# 2. THE DRAWABLE CANVAS WIDGET (Dynamic Ratio Upgrade)
# ---------------------------------------------------------
class DrawableCanvas(QWidget):
    def __init__(self, phys_width_m=0.30, phys_height_m=0.20):
        super().__init__()
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMinimumSize(300, 200) 
        
        self.phys_w = phys_width_m
        self.phys_h = phys_height_m
        self.target_ratio = self.phys_w / self.phys_h
        
        # --- NEW: Store multiple lines ---
        self.lines = []         # A list containing multiple separate lines
        self.current_line = []  # The specific line currently being drawn
        
        self.is_drawing = False
        self.active_rect = QRectF() 

        # Aesthetics
        self.bg_color = QColor(60, 62, 66)           
        self.border_color = QColor(120, 120, 125)    
        self.drawing_color = QColor(255, 255, 255)
        self.line_thickness = 4

    def resizeEvent(self, event):
        # Every time the window changes size, recalculate the aspect ratio boundary
        w = self.width()
        h = self.height()
        current_ratio = w / h
        
        # Calculate maximum possible size while keeping the exact physical ratio
        pad = 20 # 10px padding on all sides
        if current_ratio > self.target_ratio:
            # Widget is too wide, height dictates the scale
            draw_h = h - pad
            draw_w = draw_h * self.target_ratio
        else:
            # Widget is too tall, width dictates the scale
            draw_w = w - pad
            draw_h = draw_w / self.target_ratio
            
        # Center the rectangle in the widget
        x = (w - draw_w) / 2.0
        y = (h - draw_h) / 2.0
        self.active_rect = QRectF(x, y, draw_w, draw_h)
        super().resizeEvent(event)

    def mousePressEvent(self, event):
        # Only start drawing if they click INSIDE the active bounded box
        if event.button() == Qt.LeftButton and self.active_rect.contains(event.position()):
            self.is_drawing = True
            self.current_line = [event.position()]
            self.lines.append(self.current_line)
            self.update()

    def mouseMoveEvent(self, event):
        if self.is_drawing:
            # Prevent dragging the pen outside the canvas bounds
            pos = event.position()
            if self.active_rect.contains(pos):
                self.current_line.append(pos)
            else:
                # Clamp it to the edge if they slide off
                clamped_x = max(self.active_rect.left(), min(pos.x(), self.active_rect.right()))
                clamped_y = max(self.active_rect.top(), min(pos.y(), self.active_rect.bottom()))
                self.current_line.append(QPointF(clamped_x, clamped_y))
            self.update()

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.is_drawing = False

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # 1. Draw the bounded drawing pad
        painter.fillRect(self.active_rect, self.bg_color)
        
        # Draw the border
        border_pen = QPen(self.border_color, 3, Qt.DashLine)
        painter.setPen(border_pen)
        painter.drawRoundedRect(self.active_rect, 10, 10)

        traj_pen = QPen(self.drawing_color, self.line_thickness, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin)
        painter.setPen(traj_pen)

        for line in self.lines:
            if len(line) > 1:
                for i in range(1, len(line)):
                    painter.drawLine(line[i - 1], line[i])

    def get_points(self, hover_height_m=0.01):
        """ Returns a 1D sequence of (X, Y, Z) tuples including lift and plunge commands """
        trajectory = []
        
        # Guard against empty drawings
        if not self.lines:
            return trajectory
            
        cx = self.active_rect.center().x()
        cy = self.active_rect.center().y()
        
        # Dynamic scale factor (Meters per Pixel)
        scale_x = self.phys_w / self.active_rect.width()
        scale_y = self.phys_h / self.active_rect.height()
        
        for line in self.lines:
            if not line:
                continue
                
            # Convert this specific line's pixels to meters
            line_m = [( (pt.x() - cx) * scale_x, -(pt.y() - cy) * scale_y ) for pt in line]
            
            start_x, start_y = line_m[0]
            end_x, end_y = line_m[-1]
            
            # 1. Hover above the start of the line
            trajectory.append((start_x, start_y, hover_height_m))
            
            # 2. Plunge down to touch the board
            trajectory.append((start_x, start_y, 0.0))
            
            # 3. Draw the actual line on the board
            for pt in line_m:
                trajectory.append((pt[0], pt[1], 0.0))
                
            # 4. Lift the pen straight up at the end of the line
            trajectory.append((end_x, end_y, hover_height_m))
            
        return trajectory


# ---------------------------------------------------------
# 3. MAIN WINDOW GUI
# ---------------------------------------------------------
class MasterDashboard(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Dynamic Tracking GUI")
        self.resize(1300, 850)
        
        # READ YAML CONFIGURATION
        self.canvas_w = 0.30
        self.canvas_h = 0.20
        try:
            bringup_dir = get_package_share_directory('surface_tracking_bringup')
            yaml_path = os.path.join(bringup_dir, 'config', 'experiment_config.yaml')
            with open(yaml_path, 'r') as file:
                config = yaml.safe_load(file)['global_experiment_manager']['ros__parameters']
                self.canvas_w = float(config['canvas_width'])
                self.canvas_h = float(config['canvas_height'])
                print(f"Loaded Canvas Dimensions: {self.canvas_w}m x {self.canvas_h}m")
        except Exception as e:
            print(f"Warning: Failed to load YAML. Using defaults. Error: {e}")

        self.worker = RosWorker()
        self.worker.start()
        
        # Connect Signals
        self.worker.ee_pose_signal.connect(self.update_ee_display)
        self.worker.service_response_signal.connect(self.update_result_log)
        self.worker.servo_state_signal.connect(self.update_servo_state)
        self.worker.fault_state_signal.connect(self.update_fault_state)
        self.worker.joint_state_signal.connect(self.update_joint_display)
        self.worker.servo_status_signal.connect(self.update_servo_status)
        self.worker.action_feedback_signal.connect(self.update_action_feedback)
        self.worker.action_result_signal.connect(self.on_action_finished)
        
        self.joint_limits = [(-180, 180), (-135, 135), (-150, 150), (-180, 180), (-147, 147), (-180, 180)]
        self.vel_sliders_ui = []
        
        # Jogging (Button) State
        self.jog_timer = QTimer()
        self.jog_timer.setInterval(50)
        self.jog_timer.timeout.connect(self.jog_loop)
        self.active_jog = None

        self.setup_ui()

    def setup_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)

        # --- 1. TOP BAR: Hardware Control ---
        hw_group = QGroupBox("Hardware Control")
        hw_layout = QHBoxLayout()
        buttons = [("SERVO ON", "#63E363", 'enable'), ("SERVO OFF", "#FC657C", 'disable'),
                   ("CLEAR FAULT", None, 'clear'), ("HOME", "#FFD700", 'home'), ("STOP", "#FF4500", 'stop')]
        for label, color, cmd in buttons:
            btn = QPushButton(label)
            if color: btn.setStyleSheet(f"background-color: {color}; color: black; font-weight: bold; padding: 10px;")
            else: btn.setStyleSheet("padding: 10px;")
            btn.clicked.connect(lambda _, c=cmd: self.worker.call_service(c))
            hw_layout.addWidget(btn)
        hw_group.setLayout(hw_layout)
        main_layout.addWidget(hw_group)

        # --- 2. MIDDLE: Split Interface ---
        middle_layout = QHBoxLayout()
        
        # Left Side (Tabs)
        self.tabs = QTabWidget()
        self.tabs.addTab(self.create_canvas_tab(), "Experiment Canvas")
        self.tabs.addTab(self.create_cartesian_goal_tab(), "Cartesian Goal")
        self.tabs.addTab(self.create_joint_goal_tab(), "Joint Goal")
        self.tabs.addTab(self.create_jogging_tab(), "Jogging")
        self.tabs.addTab(self.create_tf_tab(), "Set Origin")
        middle_layout.addWidget(self.tabs, stretch=2)

        # Right Side (Status Monitor)
        middle_layout.addWidget(self.create_right_panel(), stretch=1)
        
        main_layout.addLayout(middle_layout)

        # --- 3. BOTTOM BAR: Log ---
        log_group = QGroupBox("Result Message")
        log_layout = QVBoxLayout()
        self.result_log = QLineEdit("System Ready."); self.result_log.setReadOnly(True)
        self.result_log.setAlignment(Qt.AlignCenter); self.result_log.setStyleSheet("font-size: 14px; padding: 5px;")
        log_layout.addWidget(self.result_log)
        log_group.setLayout(log_layout)
        main_layout.addWidget(log_group)

    # =========================================================================
    # LEFT PANEL TABS
    # =========================================================================
    def create_canvas_tab(self):
        w = QWidget()
        layout = QVBoxLayout(w)
        
        # 1. THE CANVAS (Eats 100% of available space)
        self.canvas = DrawableCanvas(phys_width_m=self.canvas_w, phys_height_m=self.canvas_h)
        # Adding stretch=1 here guarantees it pushes everything else to the bottom
        layout.addWidget(self.canvas, stretch=1) 

        # 2. STATUS BAR (Static Height)
        status_layout = QVBoxLayout()
        self.lbl_action_status = QLabel("Status: Idle")
        self.lbl_action_status.setAlignment(Qt.AlignCenter)
        self.bar_action_progress = QProgressBar()
        self.bar_action_progress.setRange(0, 100)
        self.bar_action_progress.setFixedHeight(25) # Lock height
        
        status_layout.addWidget(self.lbl_action_status)
        status_layout.addWidget(self.bar_action_progress)
        layout.addLayout(status_layout)

        # 3. BOTTOM BUTTONS (Static Height, Locked to Bottom)
        btn_layout = QHBoxLayout()
        
        self.btn_clear = QPushButton("CLEAR CANVAS")
        self.btn_clear.setFixedHeight(40) # Lock height
        self.btn_clear.clicked.connect(lambda: [self.canvas.lines.clear(), self.canvas.update()])
        
        self.btn_ready = QPushButton("READY (HOVER)")
        self.btn_ready.setStyleSheet("background-color: #87CEFA; color: black; font-weight: bold;")
        self.btn_ready.setFixedHeight(40) # Lock height
        self.btn_ready.clicked.connect(lambda: self.worker.set_tracking_ready_mode(True))

        self.btn_start = QPushButton("START TRAJECTORY")
        self.btn_start.setStyleSheet("background-color: #63E363; color: black; font-weight: bold;")
        self.btn_start.setFixedHeight(40) # Lock height
        self.btn_start.clicked.connect(self.trigger_robot)

        self.btn_stop = QPushButton("STOP TRACKING")
        self.btn_stop.setStyleSheet("background-color: #FC657C; color: black; font-weight: bold;")
        self.btn_stop.setFixedHeight(40) # Lock height
        self.btn_stop.clicked.connect(lambda: self.worker.set_tracking_ready_mode(False))

        btn_layout.addWidget(self.btn_clear)
        btn_layout.addWidget(self.btn_ready)
        btn_layout.addWidget(self.btn_start)
        btn_layout.addWidget(self.btn_stop)
        
        layout.addLayout(btn_layout)
        
        # Notice we removed `layout.addStretch()` from the very bottom.
        # This prevents the layout from creating empty space below the buttons.
        return w

    def create_cartesian_goal_tab(self):
        w = QWidget(); l = QVBoxLayout(w)
        self.frame_group = QButtonGroup(self)
        self.rb_local = QRadioButton("Local Frame"); self.rb_local.setChecked(True)
        self.rb_global = QRadioButton("Global Frame")
        self.frame_group.addButton(self.rb_local, 0); self.frame_group.addButton(self.rb_global, 1)
        rl = QHBoxLayout(); rl.addWidget(self.rb_local); rl.addWidget(self.rb_global); l.addLayout(rl)
        
        self.inputs = {}
        g = QGridLayout()
        lbs = ["X", "Y", "Z", "Qx", "Qy", "Qz", "Qw"]
        defs = ["0.50", "0.40", "0.30", "0.00", "1.00", "0.00", "0.00"]
        for i, (lb, v) in enumerate(zip(lbs, defs)):
            g.addWidget(QLabel(lb+":"), i, 0)
            self.inputs[lb] = QLineEdit(v)
            g.addWidget(self.inputs[lb], i, 1)
        l.addLayout(g)
        
        v_w, v_s = self.create_velocity_control(); self.vel_sliders_ui.append(v_s)
        l.addWidget(v_w)
        btn = QPushButton("PUBLISH CARTESIAN GOAL"); btn.setMinimumHeight(40); btn.clicked.connect(self.on_publish_cart)
        l.addWidget(btn); l.addStretch()
        return w

    def create_joint_goal_tab(self):
        w = QWidget(); l = QVBoxLayout(w)
        self.joint_sliders = []
        for i in range(6):
            hl = QHBoxLayout(); hl.addWidget(QLabel(f"J{i+1}"))
            mn, mx = self.joint_limits[i]
            s = QSlider(Qt.Horizontal); s.setRange(mn, mx); s.setValue(0)
            lbl = QLabel("0°"); lbl.setFixedWidth(40)
            s.valueChanged.connect(lambda val, lb=lbl: lb.setText(f"{val}°"))
            hl.addWidget(s); hl.addWidget(lbl)
            self.joint_sliders.append(s); l.addLayout(hl)
            
        v_w, v_s = self.create_velocity_control(); self.vel_sliders_ui.append(v_s)
        l.addWidget(v_w)
        btn = QPushButton("PUBLISH JOINT GOAL"); btn.setMinimumHeight(40); btn.clicked.connect(self.on_publish_joints)
        l.addWidget(btn); l.addStretch()
        return w

    def create_jogging_tab(self):
        w = QWidget(); l = QVBoxLayout(w)
        v_w, v_s = self.create_velocity_control(); self.vel_sliders_ui.append(v_s)
        l.addWidget(v_w)

        gb_c = QGroupBox("Cartesian Jog"); vl_c = QVBoxLayout(gb_c)
        self.jog_frame_btn_group = QButtonGroup(self)
        r_base = QRadioButton("Base Frame"); r_base.setChecked(True)
        r_ee = QRadioButton("Tool Frame")
        self.jog_frame_btn_group.addButton(r_base, 0); self.jog_frame_btn_group.addButton(r_ee, 1)
        frame_lay = QHBoxLayout(); frame_lay.addWidget(QLabel("Frame:")); frame_lay.addWidget(r_base); frame_lay.addWidget(r_ee)
        vl_c.addLayout(frame_lay)

        gl_c = QGridLayout()
        axes = ["X", "Y", "Z", "Rx", "Ry", "Rz"]
        for i, ax in enumerate(axes):
            btn_n = QPushButton(f"{ax} -"); btn_p = QPushButton(f"{ax} +")
            btn_n.pressed.connect(lambda i=i: self.start_jog('cart', i, -1.0)); btn_n.released.connect(self.stop_jog)
            btn_p.pressed.connect(lambda i=i: self.start_jog('cart', i, 1.0)); btn_p.released.connect(self.stop_jog)
            gl_c.addWidget(QLabel(ax), i, 0); gl_c.addWidget(btn_n, i, 1); gl_c.addWidget(btn_p, i, 2)
        vl_c.addLayout(gl_c); l.addWidget(gb_c)

        gb_j = QGroupBox("Joint Jog"); gl_j = QGridLayout()
        for i in range(6):
            btn_n = QPushButton(f"J{i+1} -"); btn_p = QPushButton(f"J{i+1} +")
            btn_n.pressed.connect(lambda i=i: self.start_jog('joint', i, -1.0)); btn_n.released.connect(self.stop_jog)
            btn_p.pressed.connect(lambda i=i: self.start_jog('joint', i, 1.0)); btn_p.released.connect(self.stop_jog)
            gl_j.addWidget(QLabel(f"J{i+1}"), i, 0); gl_j.addWidget(btn_n, i, 1); gl_j.addWidget(btn_p, i, 2)
        gb_j.setLayout(gl_j); l.addWidget(gb_j)
        l.addStretch()
        return w

    def create_tf_tab(self):
        w = QWidget(); l = QVBoxLayout(w)
        self.tf_inputs = {}
        g = QGridLayout()
        lbs = ["X", "Y", "Z", "Qx", "Qy", "Qz", "Qw"]
        defs = ["0.00", "1.00", "0.00", "0.00", "0.00", "0.00", "1.00"]
        for i, (lb, v) in enumerate(zip(lbs, defs)):
            g.addWidget(QLabel(lb+":"), i, 0)
            self.tf_inputs[lb] = QLineEdit(v)
            g.addWidget(self.tf_inputs[lb], i, 1)
        l.addLayout(g)
        btn = QPushButton("PUBLISH TF"); btn.setMinimumHeight(40); btn.clicked.connect(self.on_publish_tf)
        l.addWidget(btn); l.addStretch()
        return w

    # =========================================================================
    # RIGHT PANEL (MONITOR)
    # =========================================================================
    def create_right_panel(self):
        w = QWidget(); l = QVBoxLayout(w); l.setContentsMargins(0,0,0,0)
        
        # Robot Status
        gb_s = QGroupBox("Robot Status"); gl_s = QGridLayout()
        self.lbl_servo = QLineEdit("Unknown"); self.lbl_servo.setReadOnly(True); self.lbl_servo.setAlignment(Qt.AlignCenter)
        self.lbl_fault = QLineEdit("Unknown"); self.lbl_fault.setReadOnly(True); self.lbl_fault.setAlignment(Qt.AlignCenter)
        gl_s.addWidget(QLabel("Servo State:"), 0, 0); gl_s.addWidget(self.lbl_servo, 0, 1)
        gl_s.addWidget(QLabel("Fault State:"), 1, 0); gl_s.addWidget(self.lbl_fault, 1, 1)
        self.lbl_moveit_status = QLineEdit("Code: -"); self.lbl_moveit_status.setReadOnly(True)
        gl_s.addWidget(QLabel("MoveIt Status:"), 2, 0); gl_s.addWidget(self.lbl_moveit_status, 2, 1)
        gb_s.setLayout(gl_s); l.addWidget(gb_s)
        
        # Pose and Joints
        gb_e = QGroupBox("End-Effector Pose"); vl_e = QVBoxLayout()
        self.ee_grp = QButtonGroup(self)
        r1 = QRadioButton("Robot"); r1.setChecked(True); r2 = QRadioButton("World")
        self.ee_grp.addButton(r1, 0); self.ee_grp.addButton(r2, 1)
        self.ee_grp.idClicked.connect(lambda id: setattr(self.worker, 'current_ee_display_frame', self.worker.base_frame if id==0 else 'world'))
        rh = QHBoxLayout(); rh.addWidget(r1); rh.addWidget(r2); vl_e.addLayout(rh)
        
        self.ee_disp = {}
        ge = QGridLayout()
        for i, lb in enumerate(["X", "Y", "Z", "Qx", "Qy", "Qz", "Qw"]):
            ge.addWidget(QLabel(lb+":"), i, 0)
            self.ee_disp[lb] = QLineEdit(); self.ee_disp[lb].setReadOnly(True)
            ge.addWidget(self.ee_disp[lb], i, 1)
        vl_e.addLayout(ge); gb_e.setLayout(vl_e); l.addWidget(gb_e)

        gb_j = QGroupBox("Joint State (deg)"); gj = QGridLayout()
        self.j_disp_widgets = []
        for i in range(6):
            gj.addWidget(QLabel(f"J{i+1}:"), i, 0)
            le = QLineEdit("0.00"); le.setReadOnly(True)
            self.j_disp_widgets.append(le)
            gj.addWidget(le, i, 1)
        gb_j.setLayout(gj); l.addWidget(gb_j)
        
        l.addStretch()
        return w

    # =========================================================================
    # LOGIC HELPERS & SLOTS
    # =========================================================================
    def create_velocity_control(self):
        c = QWidget(); h = QHBoxLayout(c); h.setContentsMargins(0, 5, 0, 5)
        s = QSlider(Qt.Horizontal); s.setRange(1, 100); s.setValue(40)
        l = QLabel("40%"); l.setFixedWidth(40)
        s.valueChanged.connect(lambda v: l.setText(f"{v}%")); s.valueChanged.connect(self.sync_velocity_sliders)
        h.addWidget(QLabel("Velocity Scale:")); h.addWidget(s); h.addWidget(l)
        return c, s

    def sync_velocity_sliders(self, value):
        self.worker.current_vel_scale = float(value) / 100.0
        for s in self.vel_sliders_ui:
            if s.value() != value:
                s.blockSignals(True); s.setValue(value); s.blockSignals(False)
        self.worker.publish_velocity_scale(value)

    # --- Jogging Logic ---
    def start_jog(self, jtype, axis, direction):
        frame = self.worker.base_frame
        if jtype == 'cart' and self.jog_frame_btn_group.checkedId() == 1: frame = self.worker.ee_frame
        self.active_jog = (jtype, axis, direction, frame)
        self.jog_timer.start()

    def stop_jog(self):
        self.jog_timer.stop()
        if self.active_jog:
            jtype, axis, _, frame = self.active_jog
            if jtype == 'cart': self.worker.send_cartesian_jog(axis, 0.0, frame)
            else: self.worker.send_joint_jog(axis, 0.0)
        self.active_jog = None

    def jog_loop(self):
        if not self.active_jog: return
        jtype, axis, direction, frame = self.active_jog
        scaled_val = direction * self.worker.current_vel_scale
        if jtype == 'cart': self.worker.send_cartesian_jog(axis, scaled_val, frame)
        else: self.worker.send_joint_jog(axis, scaled_val)

    # --- Publishing Handlers ---
    def on_publish_cart(self):
        try: vals = [float(self.inputs[k].text()) for k in ["X","Y","Z","Qx","Qy","Qz","Qw"]]
        except: return
        f = self.worker.base_frame if self.frame_group.checkedId() == 0 else 'world'
        self.worker.publish_goal(vals[:3], vals[3:], f)

    def on_publish_joints(self):
        self.worker.publish_joint_goal([s.value() for s in self.joint_sliders])

    def on_publish_tf(self):
        try: vals = [float(self.tf_inputs[k].text()) for k in ["X","Y","Z","Qx","Qy","Qz","Qw"]]
        except: return
        self.worker.publish_static_tf(vals[:3], vals[3:])

    # --- Canvas & Action Logic ---
    def trigger_robot(self):
        points = self.canvas.get_points()
        if not points:
            self.update_result_log(False, "Canvas is empty!")
            return
        
        self.btn_start.setEnabled(False)
        self.lbl_action_status.setText("Status: Sending Goal...")
        self.worker.send_drawing_goal(points, scale_factor=0.0004) # 20cm / 500px

    @Slot(float, str)
    def update_action_feedback(self, pct, state):
        self.bar_action_progress.setValue(int(pct))
        self.lbl_action_status.setText(f"Status: {state}")

    @Slot(bool, str)
    def on_action_finished(self, success, msg):
        self.btn_start.setEnabled(True)
        self.update_result_log(success, msg)
        if success:
            self.bar_action_progress.setValue(100)
            self.lbl_action_status.setText("Status: Complete")
        else:
            self.lbl_action_status.setText("Status: Failed/Canceled")

    # --- Status Updating Slots ---
    @Slot(float, float, float, float, float, float, float)
    def update_ee_display(self, x, y, z, qx, qy, qz, qw):
        for k, val in zip(["X","Y","Z","Qx","Qy","Qz","Qw"], [x,y,z,qx,qy,qz,qw]): 
            self.ee_disp[k].setText(f"{val:.3f}")

    @Slot(list)
    def update_joint_display(self, vals):
        for i, val in enumerate(vals): self.j_disp_widgets[i].setText(f"{val:.1f}")

    @Slot(bool, str)
    def update_result_log(self, s, m):
        self.result_log.setText(m)
        self.result_log.setStyleSheet(f"background-color: {'#C8E1C8' if s else '#E1C8C8'};")

    @Slot(bool)
    def update_servo_state(self, e):
        self.lbl_servo.setText("ENABLED" if e else "DISABLED")
        self.lbl_servo.setStyleSheet(f"background-color: {'#90EE90' if e else '#FFB6C1'}; font-weight: bold;")

    @Slot(bool)
    def update_fault_state(self, f):
        self.lbl_fault.setText("FAULT" if f else "NO FAULT")
        self.lbl_fault.setStyleSheet(f"background-color: {'#FF4500' if f else '#90EE90'}; font-weight: bold;")

    @Slot(int)
    def update_servo_status(self, code):
        text = {0: 'Ready', 1: 'No Warning', 2: 'Collision!', 3: 'Collision!', 4: 'Singularity'}.get(code, f"Code: {code}")
        self.lbl_moveit_status.setText(text)
        if code == 0: self.lbl_moveit_status.setStyleSheet("background-color: #90EE90; color: black;")
        elif code in [1, 3]: self.lbl_moveit_status.setStyleSheet("background-color: #FFD700; color: black;")
        else: self.lbl_moveit_status.setStyleSheet("background-color: #FF4500; color: white")

    def closeEvent(self, event):
        self.worker.quit()
        event.accept()

# ---------------------------------------------------------
# STARTUP
# ---------------------------------------------------------
def main(args=None):
    app = QApplication(sys.argv)
    window = MasterDashboard()
    window.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()