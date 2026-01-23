#!/usr/bin/env python3
import sys
import os
import subprocess
import signal
import socket
import re
import time
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy
from PyQt5.QtWidgets import *
from PyQt5.QtCore import QTimer, QProcess
from PyQt5.QtGui import QIcon
from PyQt5.QtWidgets import QApplication
from ament_index_python.packages import get_package_share_directory
from qtermwidget_wrapper import QTermWidget
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool, Float32MultiArray, Float64MultiArray
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from ur_msgs.msg import IOStates
from ur_msgs.srv import SetIO
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class RobotControlUI(QMainWindow):
    def __init__(self):
        super().__init__()
 
        # Initialize ROS only if not already initialized
        if not rclpy.ok():
            rclpy.init()
            
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
 
        self.node = rclpy.create_node('robot_control_ui')
        self.goal_publisher = self.node.create_publisher(Pose, '/arm/goal_pose', 10)
        self.emergency_stop_publisher = self.node.create_publisher(Bool, '/arm/emergency_stop', qos)
        self.distance_sensor_publisher = self.node.create_publisher(Float32MultiArray, '/arm/distance_sensors', 10)
        self.emergency_stop_subscriber = self.node.create_subscription(Bool, '/arm/emergency_stop', self._on_emergency_stop_state, qos)
 
        # Track processes and their associated buttons
        self.process_map = {}
        self.button_map = {}
 
        # Store cleared status text for restore functionality
        self.cleared_status_backup = None
        self.base_cleared_status_backup = None
        
        # Emergency stop UI state
        self.emergency_stop_active = False
 
        # Robot dashboard connection
        self.robot_socket = None
        self.robot_host = '192.168.1.102'
        self.robot_port = 29999
 
        # Action client for canceling trajectory goals
        self.action_client = ActionClient(
            self.node, 
            FollowJointTrajectory, 
            '/arm/joint_trajectory_controller/follow_joint_trajectory'
        )
        self.current_goal_handle = None
 
        self.setWindowTitle("Robot Control Panel")
        self.setGeometry(100, 100, 1200, 700)
 
        # Set window icon
        try:
            pkg_share = get_package_share_directory('arm_control')
            icon_path = os.path.join(pkg_share, 'resource', 'robot_icon.png')
            if os.path.exists(icon_path):
                self.setWindowIcon(QIcon(icon_path))
        except Exception:
            # Fallback to source directory if package not found
            icon_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'resource', 'robot_icon.png')
            if os.path.exists(icon_path):
                self.setWindowIcon(QIcon(icon_path))
 
        # Central widget
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
 
        # Create tab widget for Arm Control and Base Control
        tabs = QTabWidget()
        main_layout.addWidget(tabs)
 
        # ===== ARM CONTROL TAB =====
        arm_tab = QWidget()
        arm_tab_layout = QVBoxLayout(arm_tab)
 
        # Simulation parameter selector (at top)
        arm_sim_param_layout = QHBoxLayout()
        arm_sim_param_layout.addWidget(QLabel("Simulation Mode:"))
        self.arm_sim_mode_combo = QComboBox()
        self.arm_sim_mode_combo.addItems(['false', 'true'])
        arm_sim_param_layout.addWidget(self.arm_sim_mode_combo)
        arm_sim_param_layout.addStretch()
        arm_tab_layout.addLayout(arm_sim_param_layout)
        
        # Horizontal layout for three boxes side by side
        boxes_layout = QHBoxLayout()
        arm_tab_layout.addLayout(boxes_layout)
        
        # Control Box
        control_box = QGroupBox("Control")
        control_layout = QVBoxLayout()
        control_box.setLayout(control_layout)
 
        # Launch General button
        control_layout.addWidget(QLabel("System Control:"))
        self.btn_general_launch = QPushButton("Launch General")
        self.btn_general_launch.clicked.connect(self.toggle_general_launch)
        self.btn_general_launch.setToolTip("ros2 launch arm_control general.launch.py use_fake_hardware:=false robot_ip:=192.168.1.102 initial_joint_controller:=joint_trajectory_controller")
        control_layout.addWidget(self.btn_general_launch)
 
        # RQT Joint Controller button
        self.btn_rqt_controller = QPushButton("Launch RQT Joint Controller")
        self.btn_rqt_controller.clicked.connect(self.toggle_rqt_controller)
        self.btn_rqt_controller.setToolTip("ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller --force-discover --ros-args -r __ns:=/arm\n(Requires General Launch to be running)")
        control_layout.addWidget(self.btn_rqt_controller)
 
        # Dropdown for position selection
        control_layout.addWidget(QLabel("Select Position:"))
        position_sender_layout = QHBoxLayout()
        self.position_dropdown = QComboBox()
        self.position_dropdown.addItems(['custom', 'folded', 'unfolded', 'up', 'down', 'front', 'list'])
        position_sender_layout.addWidget(self.position_dropdown)
 
        btn_send_position = QPushButton("Send Position")
        btn_send_position.clicked.connect(self.send_position_command)
        btn_send_position.setToolTip("ros2 run arm_control position_sender_node --ros-args -r __ns:=/arm -- <position>")
        position_sender_layout.addWidget(btn_send_position)
        control_layout.addLayout(position_sender_layout)
 
        # List Controllers button
        btn_list_controllers = QPushButton("List Controllers")
        btn_list_controllers.clicked.connect(self.list_controllers)
        btn_list_controllers.setToolTip("ros2 control list_controllers -c /arm/controller_manager")
        control_layout.addWidget(btn_list_controllers)
 
        # Emergency stop button
        goal_layout = QHBoxLayout()
 
        self.btn_emergency_stop = QPushButton("EMERGENCY STOP (Click to Activate)")
        self.btn_emergency_stop.setCheckable(True)
        self.btn_emergency_stop.setStyleSheet("background-color: red; color: white; font-weight: bold;")
        self.btn_emergency_stop.clicked.connect(self.emergency_stop)
        self.btn_emergency_stop.setToolTip("Toggle emergency stop on /arm/emergency_stop and cancel trajectory goals when activating")
        goal_layout.addWidget(self.btn_emergency_stop)
        control_layout.addLayout(goal_layout)
 
        control_layout.addStretch()
 
        # Initialization Box (formerly Status, now first)
        init_box = QGroupBox("Initialization")
        init_layout = QVBoxLayout()
        init_box.setLayout(init_layout)
 
        # Status Commands
        init_layout.addWidget(QLabel("Status Commands:"))
        self.status_cmd_combo = QComboBox()
        self.status_cmd_combo.addItems([
            'robotmode',
            'safetystatus',
            'programState',
            'running',
            'get loaded program',
            'is in remote control'
        ])
        init_layout.addWidget(self.status_cmd_combo)
 
        btn_send_status = QPushButton("Send Status Command")
        btn_send_status.clicked.connect(self.send_status_command)
        init_layout.addWidget(btn_send_status)
 
        # Control Commands
        init_layout.addWidget(QLabel("\nControl Commands:"))
        self.control_cmd_combo = QComboBox()
        self.control_cmd_combo.addItems([
            'power on',
            'power off',
            'brake release',
            'play',
            'pause',
            'shutdown',
            'stop',
            'close popup',
            'restart safety',
            'load Test_external_control.urp',
            'close safety popup',
            'unlock protective stop'
        ])
        init_layout.addWidget(self.control_cmd_combo)
 
        btn_send_control = QPushButton("Send Control Command")
        btn_send_control.clicked.connect(self.send_control_command)
        init_layout.addWidget(btn_send_control)
 
        init_layout.addStretch()
        boxes_layout.addWidget(init_box)
 
        # Control Box (now second)
        boxes_layout.addWidget(control_box)
 
        # Sensors Box
        sensors_box = QGroupBox("Sensors")
        sensors_layout = QVBoxLayout()
        sensors_box.setLayout(sensors_layout)
 
        # Arduino sensor node button
        self.btn_arduino_sensors = QPushButton("Launch Arduino Sensors")
        self.btn_arduino_sensors.clicked.connect(self.toggle_arduino_sensors)
        self.btn_arduino_sensors.setToolTip("ros2 run arm_control sensors_orientation_arduino --ros-args -r __ns:=/arm")
        sensors_layout.addWidget(self.btn_arduino_sensors)
 
        # Sensors orientation node button
        self.btn_sensors_orientation = QPushButton("Launch Sensors Orientation")
        self.btn_sensors_orientation.clicked.connect(self.toggle_sensors_orientation)
        self.btn_sensors_orientation.setToolTip("ros2 run arm_control sensors_orientation --ros-args -r __ns:=/arm")
        sensors_layout.addWidget(self.btn_sensors_orientation)
 
        sensors_layout.addStretch()
        boxes_layout.addWidget(sensors_box)
 
# Terminal and Status for Arm Control tab (side by side)
        arm_terminal_status_layout = QHBoxLayout()
        
        # Left side: Terminal
        self.arm_terminal = QTermWidget()
        arm_terminal_status_layout.addWidget(self.arm_terminal)
        
        # Right side: Status display
        self.status_text = QTextEdit()
        self.status_text.setReadOnly(True)
        self.status_text.setAcceptRichText(True)
        self.status_text.setStyleSheet("background-color: #22272e; color: #adbac7; border: 1px solid #444c56; font-family: 'Courier New', monospace; white-space: pre;")
        
        # Set tab stops to 8 characters (standard terminal width)
        from PyQt5.QtGui import QFontMetrics
        font_metrics = QFontMetrics(self.status_text.font())
        tab_width = font_metrics.horizontalAdvance(' ') * 8
        self.status_text.setTabStopDistance(tab_width)
        arm_terminal_status_layout.addWidget(self.status_text)

        status_header = QHBoxLayout()
        status_header.addWidget(QLabel("sudo apt install libqtermwidget5-0-dev qtermwidget5-data"))
        status_header.addStretch()

        # Search bar
        status_header.addWidget(QLabel("Search:"))
        self.search_input = QLineEdit()
        self.search_input.setPlaceholderText("Enter search term...")
        self.search_input.setMaximumWidth(150)
        self.search_input.returnPressed.connect(lambda: self.search_status(forward=True))
        status_header.addWidget(self.search_input)

        btn_search_prev = QPushButton("◀")
        btn_search_prev.clicked.connect(lambda: self.search_status(forward=False))
        btn_search_prev.setMaximumWidth(40)
        btn_search_prev.setToolTip("Find previous")
        status_header.addWidget(btn_search_prev)

        btn_search_next = QPushButton("▶")
        btn_search_next.clicked.connect(lambda: self.search_status(forward=True))
        btn_search_next.setMaximumWidth(40)
        btn_search_next.setToolTip("Find next")
        status_header.addWidget(btn_search_next)

        self.btn_restore_status = QPushButton("Restore")
        self.btn_restore_status.clicked.connect(self.restore_status)
        self.btn_restore_status.setMaximumWidth(80)
        self.btn_restore_status.setVisible(False)
        status_header.addWidget(self.btn_restore_status)

        btn_clear_status = QPushButton("Clear")
        btn_clear_status.clicked.connect(self.clear_status)
        btn_clear_status.setMaximumWidth(80)
        status_header.addWidget(btn_clear_status)
        arm_tab_layout.addLayout(status_header)

        arm_tab_layout.addLayout(arm_terminal_status_layout)
 
        # ===== BASE CONTROL TAB =====
        base_tab = QWidget()
        base_tab_layout = QVBoxLayout(base_tab)
 
        # Horizontal layout for control boxes side by side
        base_boxes_layout = QHBoxLayout()
        base_tab_layout.addLayout(base_boxes_layout)
 
        # Simulation parameter selector (at top)
        sim_param_layout = QHBoxLayout()
        sim_param_layout.addWidget(QLabel("Simulation Mode:"))
        self.sim_mode_combo = QComboBox()
        self.sim_mode_combo.addItems(['false', 'true'])
        sim_param_layout.addWidget(self.sim_mode_combo)
        sim_param_layout.addStretch()
        base_tab_layout.insertLayout(0, sim_param_layout)
 
        # Mapping and Localization Box
        mapping_loc_box = QGroupBox("Mapping and Localization")
        mapping_loc_layout = QVBoxLayout()
        mapping_loc_box.setLayout(mapping_loc_layout)
 
        # Launch Mapping button
        self.btn_launch_mapping = QPushButton("Launch Mapping")
        self.btn_launch_mapping.clicked.connect(self.toggle_mapping)
        self.btn_launch_mapping.setToolTip("ros2 launch navi_wall mapping_3d.launch.py sim:=<mode> lidar:=sick")
        mapping_loc_layout.addWidget(self.btn_launch_mapping)
 
        # Launch Localization button
        self.btn_launch_localization = QPushButton("Launch Localization")
        self.btn_launch_localization.clicked.connect(self.toggle_localization)
        self.btn_launch_localization.setToolTip("ros2 launch navi_wall move_robot.launch.py sim:=<mode>")
        mapping_loc_layout.addWidget(self.btn_launch_localization)
 
        # View map button
        self.btn_view_map = QPushButton("View map")
        self.btn_view_map.clicked.connect(self.toggle_view_map)
        self.btn_view_map.setToolTip("rtabmap-databaseViewer rtabmap.db")
        mapping_loc_layout.addWidget(self.btn_view_map)
 
        mapping_loc_layout.addStretch()
        base_boxes_layout.addWidget(mapping_loc_box)
 
        # Navigation and Exploration Box
        nav_explore_box = QGroupBox("Navigation and Exploration")
        nav_explore_layout = QVBoxLayout()
        nav_explore_box.setLayout(nav_explore_layout)
 
        # Launch Nav2 button
        self.btn_launch_nav2 = QPushButton("Launch Nav2")
        self.btn_launch_nav2.clicked.connect(self.toggle_nav2)
        self.btn_launch_nav2.setToolTip("ros2 launch navi_wall navigation_launch.py use_sim_time:=<mode>")
        nav_explore_layout.addWidget(self.btn_launch_nav2)
 
        # Launch Exploration button
        self.btn_launch_exploration = QPushButton("Launch Exploration(explore_lite)")
        self.btn_launch_exploration.clicked.connect(self.toggle_exploration)
        self.btn_launch_exploration.setToolTip("ros2 run navi_wall explore --ros-args --params-file <pkg>/config/explore_params.yaml")
        nav_explore_layout.addWidget(self.btn_launch_exploration)
 
        nav_explore_layout.addStretch()
        base_boxes_layout.addWidget(nav_explore_box)
 
        # Troubleshooting Box
        troubleshooting_box = QGroupBox("Troubleshooting")
        troubleshooting_layout = QVBoxLayout()
        troubleshooting_box.setLayout(troubleshooting_layout)
 
        # PS AUX button
        btn_ps_ros = QPushButton("List ROS2 Processes")
        btn_ps_ros.clicked.connect(self.run_ps_ros)
        btn_ps_ros.setToolTip("ps aux | grep ros2")
        troubleshooting_layout.addWidget(btn_ps_ros)
 
        # List Controllers button
        btn_list_base_controllers = QPushButton("List Controllers")
        btn_list_base_controllers.clicked.connect(self.run_list_base_controllers)
        btn_list_base_controllers.setToolTip("ros2 control list_controllers")
        troubleshooting_layout.addWidget(btn_list_base_controllers)
 
        # Launch RQT button
        self.btn_launch_rqt = QPushButton("Launch RQT")
        self.btn_launch_rqt.clicked.connect(self.toggle_rqt)
        self.btn_launch_rqt.setToolTip("rqt")
        troubleshooting_layout.addWidget(self.btn_launch_rqt)
 
        troubleshooting_layout.addStretch()
        base_boxes_layout.addWidget(troubleshooting_box)
 
# Terminal and Status for Base Control tab (side by side)
        base_terminal_status_layout = QHBoxLayout()
        
        # Left side: Terminal
        self.base_terminal = QTermWidget()
        base_terminal_status_layout.addWidget(self.base_terminal)
        
        # Right side: Status display
        self.base_status_text = QTextEdit()
        self.base_status_text.setReadOnly(True)
        self.base_status_text.setAcceptRichText(True)
        self.base_status_text.setStyleSheet("background-color: #22272e; color: #adbac7; border: 1px solid #444c56; font-family: 'Courier New', monospace; white-space: pre;")
        
        # Set tab stops to 8 characters (standard terminal width)
        from PyQt5.QtGui import QFontMetrics
        font_metrics = QFontMetrics(self.base_status_text.font())
        tab_width = font_metrics.horizontalAdvance(' ') * 8
        self.base_status_text.setTabStopDistance(tab_width)
        base_terminal_status_layout.addWidget(self.base_status_text)

        base_status_header = QHBoxLayout()
        base_status_header.addWidget(QLabel("Terminal + Status:"))
        base_status_header.addStretch()

        # Search bar
        base_status_header.addWidget(QLabel("Search:"))
        self.base_search_input = QLineEdit()
        self.base_search_input.setPlaceholderText("Enter search term...")
        self.base_search_input.setMaximumWidth(150)
        self.base_search_input.returnPressed.connect(lambda: self.search_base_status(forward=True))
        base_status_header.addWidget(self.base_search_input)

        btn_base_search_prev = QPushButton("◀")
        btn_base_search_prev.clicked.connect(lambda: self.search_base_status(forward=False))
        btn_base_search_prev.setMaximumWidth(40)
        btn_base_search_prev.setToolTip("Find previous")
        base_status_header.addWidget(btn_base_search_prev)

        btn_base_search_next = QPushButton("▶")
        btn_base_search_next.clicked.connect(lambda: self.search_base_status(forward=True))
        btn_base_search_next.setMaximumWidth(40)
        btn_base_search_next.setToolTip("Find next")
        base_status_header.addWidget(btn_base_search_next)

        self.btn_restore_base_status = QPushButton("Restore")
        self.btn_restore_base_status.clicked.connect(self.restore_base_status)
        self.btn_restore_base_status.setMaximumWidth(80)
        self.btn_restore_base_status.setVisible(False)
        base_status_header.addWidget(self.btn_restore_base_status)

        btn_clear_base_status = QPushButton("Clear")
        btn_clear_base_status.clicked.connect(self.clear_base_status)
        btn_clear_base_status.setMaximumWidth(80)
        base_status_header.addWidget(btn_clear_base_status)
        base_tab_layout.addLayout(base_status_header)

        base_tab_layout.addLayout(base_terminal_status_layout)
 
        tabs.addTab(base_tab, "Base Control")
        tabs.addTab(arm_tab, "Arm Control")
 
        # ===== JOINT CONTROL TAB =====
        joint_tab = QWidget()
        joint_tab_layout = QVBoxLayout(joint_tab)
        
        # Joint Control Box
        joint_control_box = QGroupBox("Joint Position Control")
        joint_control_layout = QVBoxLayout()
        joint_control_box.setLayout(joint_control_layout)
        
        joint_control_layout.addWidget(QLabel("<b>Joint Positions (radians):</b>"))
        
        # Joint names in the correct order for publishing
        joint_names = [
            'arm_shoulder_pan_joint', 
            'arm_shoulder_lift_joint', 
            'arm_elbow_joint', 
            'arm_wrist_1_joint', 
            'arm_wrist_2_joint', 
            'arm_wrist_3_joint'
        ]
        
        # Create sliders and input fields for each joint
        self.joint_inputs = []
        self.joint_sliders = []
        for joint_name in joint_names:
            joint_row = QHBoxLayout()
            
            # Joint label
            label = QLabel(f"{joint_name}:")
            label.setMinimumWidth(150)
            joint_row.addWidget(label)
            
            # Slider
            slider = QSlider()
            slider.setOrientation(1)  # Horizontal
            slider.setRange(-628, 628)  # -6.28 to 6.28 in hundredths
            slider.setValue(0)
            slider.setTickPosition(QSlider.TicksBelow)
            slider.setTickInterval(100)
            joint_row.addWidget(slider)
            
            # Input field (shorter)
            input_field = QDoubleSpinBox()
            input_field.setRange(-6.28, 6.28)  # Approximately -2π to 2π
            input_field.setValue(0.0)
            input_field.setDecimals(3)
            input_field.setSingleStep(0.1)
            input_field.setSuffix(" rad")
            input_field.setMaximumWidth(120)
            input_field.setEnabled(False)  # Disabled until positions are read
            joint_row.addWidget(input_field)
            
            # Connect slider and input field
            slider.valueChanged.connect(lambda val, field=input_field: field.setValue(val / 100.0))
            input_field.valueChanged.connect(lambda val, s=slider: s.setValue(int(val * 100)))
            
            # Disable slider by default for safety
            slider.setEnabled(False)
            
            self.joint_inputs.append(input_field)
            self.joint_sliders.append(slider)
            joint_control_layout.addLayout(joint_row)
        
        # Time from start input
        time_layout = QHBoxLayout()
        time_layout.addWidget(QLabel("Time from start:"))
        self.time_from_start_input = QDoubleSpinBox()
        self.time_from_start_input.setRange(0.1, 60.0)
        self.time_from_start_input.setValue(1.0)
        self.time_from_start_input.setDecimals(1)
        self.time_from_start_input.setSingleStep(0.5)
        self.time_from_start_input.setSuffix(" sec")
        time_layout.addWidget(self.time_from_start_input)
        time_layout.addStretch()
        joint_control_layout.addLayout(time_layout)
        
        # Buttons
        button_layout = QHBoxLayout()
        
        btn_read_joints = QPushButton("Read Current Joint Positions")
        btn_read_joints.clicked.connect(self.read_joint_positions)
        btn_read_joints.setToolTip("Read current joint positions from /arm/joint_states and populate fields")
        button_layout.addWidget(btn_read_joints)
        
        self.btn_publish_joints = QPushButton("Publish Joint Trajectory")
        self.btn_publish_joints.clicked.connect(self.publish_joint_trajectory)
        self.btn_publish_joints.setToolTip("Publish joint trajectory to /arm/planned_trajectory topic")
        self.btn_publish_joints.setEnabled(False)  # Disabled until positions are read
        button_layout.addWidget(self.btn_publish_joints)
        
        joint_control_layout.addLayout(button_layout)
        
        joint_tab_layout.addWidget(joint_control_box)
        
        # Terminal and Status for Joint Control tab (side by side)
        joint_terminal_status_layout = QHBoxLayout()
        
        # Left side: Terminal
        self.joint_terminal = QTermWidget()
        joint_terminal_status_layout.addWidget(self.joint_terminal)
        
        # Right side: Status display
        self.joint_status_text = QTextEdit()
        self.joint_status_text.setReadOnly(True)
        self.joint_status_text.setAcceptRichText(True)
        self.joint_status_text.setStyleSheet("background-color: #22272e; color: #adbac7; border: 1px solid #444c56; font-family: 'Courier New', monospace; white-space: pre;")
        
        # Set tab stops to 8 characters (standard terminal width)
        from PyQt5.QtGui import QFontMetrics
        font_metrics = QFontMetrics(self.joint_status_text.font())
        tab_width = font_metrics.horizontalAdvance(' ') * 8
        self.joint_status_text.setTabStopDistance(tab_width)
        joint_terminal_status_layout.addWidget(self.joint_status_text)
        
        joint_status_header = QHBoxLayout()
        joint_status_header.addWidget(QLabel("1:sudo apt install xterm 2:apt-get install xterm 3: sudo apt install libqtermwidget5-0"))
        joint_status_header.addStretch()
        
        btn_clear_joint_status = QPushButton("Clear")
        btn_clear_joint_status.clicked.connect(self.clear_joint_status)
        btn_clear_joint_status.setMaximumWidth(80)
        joint_status_header.addWidget(btn_clear_joint_status)
        joint_tab_layout.addLayout(joint_status_header)
        
        joint_tab_layout.addLayout(joint_terminal_status_layout)
        
        tabs.addTab(joint_tab, "Joint Control")
        
        # Connect tab change signal to check joint states when Joint Control tab is activated
        tabs.currentChanged.connect(lambda index: self._on_tab_changed(index, tabs))
        
        # Timer for ROS spinning
        self.timer = QTimer()
        self.timer.timeout.connect(self._spin_ros)
        self.timer.start(100)  # 10 Hz
        
        # Initial UI update for consistent visuals
        self._update_emergency_stop_button_ui()
        # QApplication.processEvents()
 
    def _spin_ros(self):
        """Safely spin ROS, checking context is valid first"""
        try:
            if rclpy.ok():
                rclpy.spin_once(self.node, timeout_sec=0)
        except Exception:
            pass  # Ignore errors if context is shutting down
    
    def _on_tab_changed(self, index, tabs):
        """Handle tab change - check joint states when Joint Control tab is activated"""
        # Check if the Joint Control tab (index 2) is now active
        if index == 2 and tabs.tabText(index) == "Joint Control":
            # Disable controls first for safety
            for input_field in self.joint_inputs:
                input_field.setEnabled(False)
            for slider in self.joint_sliders:
                slider.setEnabled(False)
            self.btn_publish_joints.setEnabled(False)
            
            # Automatically read current joint positions to verify topic is still publishing
            self.joint_status_text.append("<span style='color: #539bf5;'>ℹ Checking for current joint positions...</span>")
            self.read_joint_positions()
 
    def _ansi_to_html(self, text):
        """Convert ANSI color codes to HTML"""
        # ANSI color code mapping
        ansi_colors = {
            '30': '#adbac7',  # black (using default text color)
            '31': '#f47067',  # red
            '32': '#57ab5a',  # green
            '33': '#c69026',  # yellow
            '34': '#539bf5',  # blue
            '35': '#b083f0',  # magenta
            '36': '#76e3ea',  # cyan
            '37': '#adbac7',  # white
            '90': '#636e7b',  # bright black (gray)
            '91': '#ff938a',  # bright red
            '92': '#6bc46d',  # bright green
            '93': '#daaa3f',  # bright yellow
            '94': '#6cb6ff',  # bright blue
            '95': '#d2a8ff',  # bright magenta
            '96': '#96d0ff',  # bright cyan
            '97': '#cdd9e5',  # bright white
        }
 
        # First, expand tabs to spaces (8-char tab stops like terminal)
        expanded_text = ''
        col = 0
        for char in text:
            if char == '\t':
                # Calculate spaces needed to reach next 8-char boundary
                spaces_needed = 8 - (col % 8)
                expanded_text += ' ' * spaces_needed
                col += spaces_needed
            elif char == '\033':
                # ANSI escape sequence doesn't affect column position
                expanded_text += char
            else:
                expanded_text += char
                if char not in '\033\r':  # Don't count escape sequences
                    col += 1
 
        # Now process ANSI codes
        result = []
        current_color = None
 
        # Split by ANSI codes
        parts = re.split(r'\033\[([0-9;]+)m', expanded_text)
 
        for i, part in enumerate(parts):
            if i % 2 == 0:  # Text content
                if part:
                    # Escape HTML special chars
                    escaped_part = part.replace('&', '&amp;').replace('<', '&lt;').replace('>', '&gt;')
                    if current_color:
                        result.append(f"<span style='color: {current_color};'>{escaped_part}</span>")
                    else:
                        result.append(escaped_part)
            else:  # ANSI code
                codes = part.split(';')
                if '0' in codes or not codes[0]:  # Reset
                    current_color = None
                else:
                    # Look for color code
                    for code in codes:
                        if code in ansi_colors:
                            current_color = ansi_colors[code]
                            break
 
        return ''.join(result)
 
    def toggle_general_launch(self):
        sim_mode = self.arm_sim_mode_combo.currentText()
        self._toggle_process('general_launch', self.btn_general_launch, 'General Launch',
                            'ros2', ['launch', 'arm_control', 'general.launch.py', 
                                    f'use_fake_hardware:={sim_mode}', f'arm_use_sim_time:={sim_mode}', 'robot_ip:=192.168.1.102',
                                    f'launch_rviz:=false',
                                    'initial_joint_controller:=joint_trajectory_controller'
                                    ])
 
    def toggle_rqt_controller(self):
        self._toggle_process('rqt_controller', self.btn_rqt_controller, 'RQT Joint Controller',
                            'ros2', ['run', 'rqt_joint_trajectory_controller', 'rqt_joint_trajectory_controller', 
                                    '--force-discover', '--ros-args', '-r', '__ns:=/arm'])
 
    def toggle_arduino_sensors(self):
        self._toggle_process('arduino_sensors', self.btn_arduino_sensors, 'Arduino Sensors',
                            'ros2', ['run', 'arm_control', 'arduino_sensors', '--ros-args', '-r', '__ns:=/arm'])
 
    def toggle_sensors_orientation(self):
        self._toggle_process('sensors_orientation', self.btn_sensors_orientation, 'Sensors Orientation',
                            'ros2', ['run', 'arm_control', 'sensors_orientation', '--ros-args', '-r', '__ns:=/arm'])
 
    def toggle_mapping(self):
        sim_mode = self.sim_mode_combo.currentText()
        # Build args based on sim mode
        if sim_mode == 'true':
            args = ['launch', 'navi_wall', 'mapping_3d.launch.py', 'sim:=true', 'lidar:=sick']
        else:
            args = ['launch', 'navi_wall', 'mapping_3d.launch.py']
 
        self._toggle_base_process('mapping', self.btn_launch_mapping, 'Mapping', 'ros2', args)
        # Disable/enable localization button based on mapping state
        if 'mapping' in self.process_map:
            self.btn_launch_localization.setEnabled(False)
        else:
            self.btn_launch_localization.setEnabled(True)
 
    def toggle_localization(self):
        sim_mode = self.sim_mode_combo.currentText()
        self._toggle_base_process('localization', self.btn_launch_localization, 'Localization',
                                 'ros2', ['launch', 'navi_wall', 'move_robot.launch.py', 
                                         f'sim:={sim_mode}'])
        # Disable/enable mapping button based on localization state
        if 'localization' in self.process_map:
            self.btn_launch_mapping.setEnabled(False)
        else:
            self.btn_launch_mapping.setEnabled(True)
 
    def toggle_view_map(self):
        # Get package path for rtabmap.db
        try:
            pkg_share = get_package_share_directory('navi_wall')
            db_path = os.path.join(pkg_share, 'maps', 'rtabmap.db')
        except Exception as e:
            self.base_status_text.append(f"Error: Could not find navi_wall package: {e}")
            return
        
        self._toggle_base_process('view_map', self.btn_view_map, 'View map',
                                 'rtabmap-databaseViewer', [db_path])
 
    def toggle_nav2(self):
        sim_mode = self.sim_mode_combo.currentText()
        self._toggle_base_process('nav2', self.btn_launch_nav2, 'Nav2',
                                 'ros2', ['launch', 'navi_wall', 'navigation_launch.py', 
                                         f'use_sim_time:={sim_mode}'])
 
    def toggle_exploration(self):
        # Get package path for explore_params.yaml
        try:
            pkg_share = get_package_share_directory('navi_wall')
            params_file = os.path.join(pkg_share, 'config', 'explore_params.yaml')
        except:
            # Fallback to source directory
            params_file = '/home/zed/ros2_ws/src/navi-wall/config/explore_params.yaml'
 
        self._toggle_base_process('exploration', self.btn_launch_exploration, 'Exploration',
                                 'ros2', ['run', 'navi_wall', 'explore', 
                                         '--ros-args', '--params-file', params_file])
 
    def toggle_rqt(self):
        """Toggle RQT on/off"""
        self._toggle_base_process('rqt', self.btn_launch_rqt, 'RQT', 'rqt', [])
 
    def run_ps_ros(self):
        """Run ps aux | grep ros2 command"""
        process = QProcess(self)
        process.setProcessChannelMode(QProcess.MergedChannels)
        process.readyReadStandardOutput.connect(lambda: self.handle_base_output(process))
 
        # Display command in bold green
        cmd_str = 'ps aux | grep ros2'
        cursor = self.base_status_text.textCursor()
        cursor.movePosition(cursor.End)
        self.base_status_text.setTextCursor(cursor)
        self.base_status_text.insertHtml(f"<b style='color: #57ab5a;'>▶ {cmd_str}</b>")
        cursor.insertText('\n')  # Ensure newline after command
 
        # Run the command using shell to support pipe
        process_key = 'ps_ros2'
        process.finished.connect(lambda: self._cleanup_ps_ros(process_key))
        process.start('bash', ['-c', 'ps aux | grep ros2'])
        self.process_map[process_key] = process
 
    def _cleanup_ps_ros(self, process_key):
        """Clean up finished ps aux process"""
        if process_key in self.process_map:
            del self.process_map[process_key]
 
    def run_list_base_controllers(self):
        """List ROS2 controllers using ros2 control CLI"""
        process = QProcess(self)
        process.setProcessChannelMode(QProcess.MergedChannels)
        process.readyReadStandardOutput.connect(lambda: self.handle_base_output(process))
 
        # Display command in bold green
        cmd_str = 'timeout 10 ros2 control list_controllers'
        cursor = self.base_status_text.textCursor()
        cursor.movePosition(cursor.End)
        self.base_status_text.setTextCursor(cursor)
        self.base_status_text.insertHtml(f"<b style='color: #57ab5a;'>▶ {cmd_str}</b>")
        cursor.insertText('\n')  # Ensure newline after command
 
        # Run the command with timeout (10 seconds)
        process_key = 'list_base_controllers'
        process.finished.connect(lambda: self._cleanup_list_base_controllers(process_key))
        process.start('timeout', ['10', 'ros2', 'control', 'list_controllers'])
        self.process_map[process_key] = process
 
    def _cleanup_list_base_controllers(self, process_key):
        """Clean up finished list controllers process"""
        if process_key in self.process_map:
            exit_code = self.process_map[process_key].exitCode()
            del self.process_map[process_key]
            if exit_code != 0:
                self.base_status_text.append(f"<span style='color: #c69026;'>⚠ List controllers command finished with exit code {exit_code}</span>")
            else:
                self.base_status_text.append("✓ List controllers command completed")
 
    def _toggle_process(self, process_key, button, name, program, args):
        """Toggle a process on/off and update button state"""
        if process_key in self.process_map:
            # Stop the process
            process = self.process_map[process_key]
            # Disconnect finished signal to prevent race condition
            try:
                process.finished.disconnect()
            except:
                pass
 
            # For launch processes, kill only child processes (including rviz2)
            if process_key in ['ur_control', 'general_launch']:
                pid = process.processId()
                if pid:
                    try:
                        # Kill all children of this specific launch process
                        subprocess.run(['pkill', '-9', '-P', str(pid)], timeout=2, stderr=subprocess.DEVNULL)
                    except:
                        pass
                # Extra cleanup for child ROS processes spawned by launch files
                self._cleanup_ros_children_of_pid(pid)
 
            process.terminate()
            process.waitForFinished(3000)
            if process.state() == QProcess.Running:
                process.kill()
 
            del self.process_map[process_key]
            button.setText(f"Launch {name}")
            button.setStyleSheet("")
            self.status_text.append(f"⏹ Stopped {name}")
        else:
            # Start the process
            process = QProcess(self)
            process.setProcessChannelMode(QProcess.MergedChannels)
            process.readyReadStandardOutput.connect(lambda: self.handle_output(process))
            process.finished.connect(lambda: self._on_process_finished(process_key, button, name))
 
            # Display command in bold green
            cmd_str = program + ' ' + ' '.join(args)
            cursor = self.status_text.textCursor()
            cursor.movePosition(cursor.End)
            self.status_text.setTextCursor(cursor)
            self.status_text.insertHtml(f"<b style='color: #57ab5a;'>▶ {cmd_str}</b>")
            cursor.insertText('\n')  # Ensure newline after command
 
            process.start(program, args)
            self.process_map[process_key] = process
            button.setText(f"Stop {name}")
            button.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;")
 
    def _toggle_base_process(self, process_key, button, name, program, args):
        """Toggle a base control process on/off and update button state (outputs to base_status_text)"""
        if process_key in self.process_map:
            # Stop the process
            process = self.process_map[process_key]
            # Disconnect finished signal to prevent race condition
            try:
                process.finished.disconnect()
            except:
                pass
 
            # For launch processes, kill child processes
            if process_key in ['mapping', 'localization', 'nav2', 'exploration']:
                # Add status message for mapping shutdown
                if process_key == 'mapping':
                    self.base_status_text.append(f"💾 Saving {name} database...")
                
                pid = process.processId()
                if pid:
                    try:
                        # Use SIGINT (same as CTRL+C) for graceful termination
                        subprocess.run(['pkill', '-INT', '-P', str(pid)], timeout=2, stderr=subprocess.DEVNULL)
                        # Wait a moment for graceful shutdown
                        time.sleep(3)
                    except:
                        pass
                self._cleanup_ros_children_of_pid(pid)

            process.terminate()  # SIGTERM - graceful
            process.waitForFinished(5000)  # Wait up to 5 seconds for graceful shutdown
            if process.state() == QProcess.Running:
                process.kill()  # SIGKILL - force kill only if still running
 
            del self.process_map[process_key]
            button.setText(f"{name}")
            button.setStyleSheet("")
            self.base_status_text.append(f"⏹ Stopped {name}")
        else:
            # Start the process
            process = QProcess(self)
            process.setProcessChannelMode(QProcess.MergedChannels)
            process.readyReadStandardOutput.connect(lambda: self.handle_base_output(process))
            process.finished.connect(lambda: self._on_base_process_finished(process_key, button, name))
 
            # Display command in bold green
            cmd_str = program + ' ' + ' '.join(args)
            cursor = self.base_status_text.textCursor()
            cursor.movePosition(cursor.End)
            self.base_status_text.setTextCursor(cursor)
            self.base_status_text.insertHtml(f"<b style='color: #57ab5a;'>▶ {cmd_str}</b>")
            cursor.insertText('\n')  # Ensure newline after command
 
            process.start(program, args)
            self.process_map[process_key] = process
            button.setText(f"Stop {name}")
            button.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;")
 
    def _on_process_finished(self, process_key, button, name):
        """Handle when a process finishes unexpectedly"""
        if process_key in self.process_map:
            del self.process_map[process_key]
            button.setText(f"Launch {name}")
            button.setStyleSheet("")
            self.status_text.append(f"<span style='color: #c69026;'>⚠ {name} exited</span>")
 
    def _on_base_process_finished(self, process_key, button, name):
        """Handle when a base process finishes unexpectedly"""
        if process_key in self.process_map:
            del self.process_map[process_key]
            button.setText(f"Launch {name}")
            button.setStyleSheet("")
            self.base_status_text.append(f"<span style='color: #c69026;'>⚠ {name} exited</span>")
 
            # Re-enable mutually exclusive buttons
            if process_key == 'mapping':
                self.btn_launch_localization.setEnabled(True)
            elif process_key == 'localization':
                self.btn_launch_mapping.setEnabled(True)
 
    def handle_output(self, process):
        output = process.readAllStandardOutput().data().decode()
        if output:
            # Don't strip output to preserve formatting (especially for YAML)
            lines = output.split('\n')
            for line in lines:
                # Skip expected shutdown messages (exit code -9 from our kill signals)
                if 'process has died' in line and 'exit code -9' in line:
                    continue
 
                # Convert ANSI color codes to HTML
                html_line = self._ansi_to_html(line)
 
                # Use insertHtml to properly render HTML entities
                cursor = self.status_text.textCursor()
                cursor.movePosition(cursor.End)
                self.status_text.setTextCursor(cursor)
                self.status_text.insertHtml(html_line)
                cursor.insertText('\n')  # Use plain text newline to preserve formatting
 
    def handle_base_output(self, process):
        """Handle output for base control processes (outputs to base_status_text)"""
        output = process.readAllStandardOutput().data().decode()
        if output:
            # Don't strip output to preserve formatting (especially for YAML)
            lines = output.split('\n')
            for line in lines:
                # Skip expected shutdown messages (exit code -9 from our kill signals)
                if 'process has died' in line and 'exit code -9' in line:
                    continue
 
                # Convert ANSI color codes to HTML
                html_line = self._ansi_to_html(line)
 
                # Use insertHtml to properly render HTML entities
                cursor = self.base_status_text.textCursor()
                cursor.movePosition(cursor.End)
                self.base_status_text.setTextCursor(cursor)
                self.base_status_text.insertHtml(html_line)
                cursor.insertText('\n')  # Use plain text newline to preserve formatting
 
    def _connect_robot_socket(self):
        """Connect to robot dashboard if not already connected"""
        if self.robot_socket is None:
            try:
                self.robot_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.robot_socket.settimeout(5)
                self.robot_socket.connect((self.robot_host, self.robot_port))
                # Read initial connection message
                data = self.robot_socket.recv(1024)
                response = data.decode('utf-8').strip()
                self.status_text.append(f"✓ Connected to robot: {response}")
                return True
            except Exception as e:
                self.status_text.append(f"<span style='color: #f47067;'>✗ Failed to connect to robot: {e}</span>")
                self.robot_socket = None
                return False
        return True
 
    def _send_robot_command(self, command):
        """Send command to robot dashboard and return response"""
        if not self._connect_robot_socket():
            return None
 
        try:
            self.robot_socket.send(str.encode(command + '\n'))
            self.status_text.append(f"<b style='color: #57ab5a;'>→ SENT: {command}</b>")
            self.status_text.append("")  # Add newline after command
 
            data = self.robot_socket.recv(1024)
            response = data.decode('utf-8').strip()
            self.status_text.append(f"← RECV: {response}")
            return response
        except Exception as e:
            self.status_text.append(f"<span style='color: #f47067;'>✗ Command failed: {e}</span>")
            # Close socket on error so it reconnects next time
            if self.robot_socket:
                self.robot_socket.close()
                self.robot_socket = None
            return None
 
    def send_status_command(self):
        """Send selected status command to robot"""
        command = self.status_cmd_combo.currentText()
        self._send_robot_command(command)
 
    def send_control_command(self):
        """Send selected control command to robot"""
        command = self.control_cmd_combo.currentText()
        self._send_robot_command(command)
 
    def list_controllers(self):
        """List ROS2 controllers using ros2 control CLI"""
        process = QProcess(self)
        process.setProcessChannelMode(QProcess.MergedChannels)
        process.readyReadStandardOutput.connect(lambda: self.handle_output(process))
 
        # Display command in bold green
        cmd_str = 'ros2 control list_controllers -c /arm/controller_manager'
        self.status_text.append(f"<b style='color: #57ab5a;'>▶ {cmd_str}</b>")
        self.status_text.append("")  # Add newline after command
 
        # Run the command
        process_key = 'list_controllers'
        process.finished.connect(lambda: self._cleanup_list_controllers(process_key))
        process.start('ros2', ['control', 'list_controllers', '-c', '/arm/controller_manager'])
        self.process_map[process_key] = process
 
    def _cleanup_list_controllers(self, process_key):
        """Clean up finished list controllers process"""
        if process_key in self.process_map:
            exit_code = self.process_map[process_key].exitCode()
            del self.process_map[process_key]
            if exit_code != 0:
                self.status_text.append(f"<span style='color: #c69026;'>⚠ List controllers command finished with exit code {exit_code}</span>")
            else:
                self.status_text.append("✓ List controllers command completed")
 
    def clear_status(self):
        """Clear status text and save backup for restore"""
        self.cleared_status_backup = self.status_text.toHtml()
        self.status_text.clear()
        self.btn_restore_status.setVisible(True)
 
    def restore_status(self):
        """Restore previously cleared status text (prepends old content like Ctrl+L undo)"""
        if self.cleared_status_backup:
            current_content = self.status_text.toHtml()
            self.status_text.setHtml(self.cleared_status_backup + current_content)
            self.cleared_status_backup = None
            self.btn_restore_status.setVisible(False)
 
    def clear_base_status(self):
        """Clear base status text and save backup for restore"""
        self.base_cleared_status_backup = self.base_status_text.toHtml()
        self.base_status_text.clear()
        self.btn_restore_base_status.setVisible(True)
 
    def restore_base_status(self):
        """Restore previously cleared base status text (prepends old content like Ctrl+L undo)"""
        if self.base_cleared_status_backup:
            current_content = self.base_status_text.toHtml()
            self.base_status_text.setHtml(self.base_cleared_status_backup + current_content)
            self.base_cleared_status_backup = None
            self.btn_restore_base_status.setVisible(False)
 
    def search_status(self, forward=True):
        """Search for text in arm status box"""
        search_text = self.search_input.text()
        if not search_text:
            return
 
        from PyQt5.QtGui import QTextDocument
        flags = QTextDocument.FindFlags()
        if not forward:
            flags |= QTextDocument.FindBackward
 
        found = self.status_text.find(search_text, flags)
        if not found:
            # Wrap around: move cursor to start/end and try again
            cursor = self.status_text.textCursor()
            if forward:
                cursor.movePosition(cursor.Start)
            else:
                cursor.movePosition(cursor.End)
            self.status_text.setTextCursor(cursor)
            self.status_text.find(search_text, flags)
 
    def search_base_status(self, forward=True):
        """Search for text in base status box"""
        search_text = self.base_search_input.text()
        if not search_text:
            return
 
        from PyQt5.QtGui import QTextDocument
        flags = QTextDocument.FindFlags()
        if not forward:
            flags |= QTextDocument.FindBackward
 
        found = self.base_status_text.find(search_text, flags)
        if not found:
            # Wrap around: move cursor to start/end and try again
            cursor = self.base_status_text.textCursor()
            if forward:
                cursor.movePosition(cursor.Start)
            else:
                cursor.movePosition(cursor.End)
            self.base_status_text.setTextCursor(cursor)
            self.base_status_text.find(search_text, flags)
   
    def clear_joint_status(self):
        """Clear joint control status text"""
        self.joint_status_text.clear()
    
    def read_joint_positions(self):
        """Read current joint positions from /arm/joint_states and populate input fields"""
        process = QProcess(self)
        process.setProcessChannelMode(QProcess.MergedChannels)
        
        # Display command in bold green
        cmd_str = 'ros2 topic echo /arm/joint_states --once'
        cursor = self.joint_status_text.textCursor()
        cursor.movePosition(cursor.End)
        self.joint_status_text.setTextCursor(cursor)
        self.joint_status_text.insertHtml(f"<b style='color: #57ab5a;'>▶ {cmd_str}</b>")
        cursor.insertText('\n')
        
        # Store the process to retrieve output later
        process.finished.connect(lambda: self._parse_joint_states(process))
        process.start('bash', ['-c', 'timeout 5 ros2 topic echo /arm/joint_states --once'])
    
    def _parse_joint_states(self, process):
        """Parse joint states output and populate input fields"""
        output = process.readAllStandardOutput().data().decode('utf-8')
        
        # Display the output line by line to preserve formatting
        lines = output.split('\n')
        for line in lines:
            html_line = self._ansi_to_html(line)
            cursor = self.joint_status_text.textCursor()
            cursor.movePosition(cursor.End)
            self.joint_status_text.setTextCursor(cursor)
            self.joint_status_text.insertHtml(html_line)
            cursor.insertText('\n')
        
        # Check if topic actually published data (not timeout or error)
        if not output.strip() or 'ERROR' in output or output.strip().startswith('timeout:'):
            self.joint_status_text.append("<span style='color: #f47067;'>✗ No data received - Topic may not be publishing yet</span>")
            self.joint_status_text.append("<span style='color: #c69026;'>⚠ Controls remain disabled for safety</span>")
            return
        
        try:
            # Parse the YAML-like output
            lines = output.split('\n')
            name_section = False
            position_section = False
            joint_names = []
            positions = []
            
            for line in lines:
                if 'name:' in line:
                    name_section = True
                    position_section = False
                    continue
                elif 'position:' in line:
                    position_section = True
                    name_section = False
                    continue
                elif 'velocity:' in line or 'effort:' in line:
                    name_section = False
                    position_section = False
                    continue
                
                if name_section and line.strip().startswith('- '):
                    joint_name = line.strip()[2:].strip()
                    joint_names.append(joint_name)
                elif position_section and line.strip().startswith('- '):
                    try:
                        position = float(line.strip()[2:].strip())
                        positions.append(position)
                    except ValueError:
                        pass
            
            # Verify we have valid data before enabling controls
            if len(joint_names) == 0 or len(positions) == 0:
                self.joint_status_text.append("<span style='color: #f47067;'>✗ No valid joint data received</span>")
                self.joint_status_text.append("<span style='color: #c69026;'>⚠ Controls remain disabled for safety</span>")
                return
            
            # Create a mapping from joint name to position
            if len(joint_names) == len(positions):
                joint_position_map = dict(zip(joint_names, positions))
                
                # Expected joint order
                expected_joints = [
                    'arm_shoulder_pan_joint', 
                    'arm_shoulder_lift_joint', 
                    'arm_elbow_joint', 
                    'arm_wrist_1_joint', 
                    'arm_wrist_2_joint', 
                    'arm_wrist_3_joint'
                ]
                
                # Verify all expected joints are present
                missing_joints = [j for j in expected_joints if j not in joint_position_map]
                if missing_joints:
                    self.joint_status_text.append(f"<span style='color: #f47067;'>✗ Missing joints: {', '.join(missing_joints)}</span>")
                    self.joint_status_text.append("<span style='color: #c69026;'>⚠ Controls remain disabled for safety</span>")
                    return
                
                # Populate input fields
                for i, joint_name in enumerate(expected_joints):
                    if joint_name in joint_position_map:
                        self.joint_inputs[i].setValue(joint_position_map[joint_name])
                
                # Store current joint positions for safety comparison
                self.current_joint_positions = [joint_position_map[j] for j in expected_joints]
                
                # Enable controls only after confirming we have valid, complete joint data
                for input_field in self.joint_inputs:
                    input_field.setEnabled(True)
                for slider in self.joint_sliders:
                    slider.setEnabled(True)
                self.btn_publish_joints.setEnabled(True)
                
                self.joint_status_text.append("<span style='color: #57ab5a;'>✓ Joint positions updated - Controls enabled</span>")
            else:
                self.joint_status_text.append("<span style='color: #f47067;'>✗ Error: Joint names and positions count mismatch</span>")
                self.joint_status_text.append("<span style='color: #c69026;'>⚠ Controls remain disabled for safety</span>")
        
        except Exception as e:
            self.joint_status_text.append(f"<span style='color: #f47067;'>✗ Error parsing joint states: {str(e)}</span>")
            self.joint_status_text.append("<span style='color: #c69026;'>⚠ Controls remain disabled for safety</span>")
    
    def publish_joint_trajectory(self):
        """Publish joint trajectory to /arm/planned_trajectory topic"""
        # Get values from input fields
        positions = [input_field.value() for input_field in self.joint_inputs]
        
        # Safety check: ensure only one joint position is different from current
        if hasattr(self, 'current_joint_positions') and self.current_joint_positions:
            differences = []
            for i, (current, requested) in enumerate(zip(self.current_joint_positions, positions)):
                # Use small tolerance for floating point comparison
                if abs(current - requested) > 0.001:
                    differences.append(i)
            
            if len(differences) > 1:
                joint_names_changed = [f"Joint {i}" for i in differences]
                self.joint_status_text.append(f"<span style='color: #c69026;'>⚠ Safety Warning: Cannot change multiple joints at once!</span>")
                self.joint_status_text.append(f"<span style='color: #c69026;'>   Changed joints: {', '.join(joint_names_changed)}</span>")
                self.joint_status_text.append(f"<span style='color: #c69026;'>   Requested joint positions should be only one for safety.</span>")
                self.joint_status_text.append("")
                return
        
        time_sec = int(self.time_from_start_input.value())
        time_nanosec = int((self.time_from_start_input.value() - time_sec) * 1e9)
        
        # Format positions for the command
        positions_str = ', '.join([f'{p:.3f}' for p in positions])
        
        # Build the ros2 topic pub command
        cmd = f"""ros2 topic pub --once /arm/planned_trajectory trajectory_msgs/msg/JointTrajectory "{{
  header: {{stamp: {{sec: 0, nanosec: 0}}, frame_id: ''}},
  joint_names: ['arm_shoulder_pan_joint', 'arm_shoulder_lift_joint', 'arm_elbow_joint', 'arm_wrist_1_joint', 'arm_wrist_2_joint', 'arm_wrist_3_joint'],
  points: [
    {{positions: [{positions_str}], velocities: [], accelerations: [], effort: [], time_from_start: {{sec: {time_sec}, nanosec: {time_nanosec}}}}}
  ]
}}\""""
        
        process = QProcess(self)
        process.setProcessChannelMode(QProcess.MergedChannels)
        process.readyReadStandardOutput.connect(lambda: self._handle_joint_publish_output(process))
        
        # Display command in bold green
        cursor = self.joint_status_text.textCursor()
        cursor.movePosition(cursor.End)
        self.joint_status_text.setTextCursor(cursor)
        self.joint_status_text.insertHtml(f"<b style='color: #57ab5a;'>▶ Publishing joint trajectory...</b>")
        cursor.insertText('\n')
        
        # Show the positions being published
        self.joint_status_text.append(f"  Positions: [{positions_str}]")
        self.joint_status_text.append(f"  Time from start: {self.time_from_start_input.value():.1f} sec")
        
        process.finished.connect(lambda: self._on_joint_publish_finished(process))
        process.start('bash', ['-c', cmd])
    
    def _handle_joint_publish_output(self, process):
        """Handle output from joint trajectory publish command"""
        output = process.readAllStandardOutput().data().decode('utf-8')
        if output:
            # Display output line by line to preserve formatting
            lines = output.split('\n')
            for line in lines:
                html_line = self._ansi_to_html(line)
                cursor = self.joint_status_text.textCursor()
                cursor.movePosition(cursor.End)
                self.joint_status_text.setTextCursor(cursor)
                self.joint_status_text.insertHtml(html_line)
                cursor.insertText('\n')
    
    def _on_joint_publish_finished(self, process):
        """Handle completion of joint trajectory publish command"""
        exit_code = process.exitCode()
        if exit_code == 0:
            self.joint_status_text.append("<span style='color: #57ab5a;'>✓ Joint trajectory published successfully</span>")
        else:
            self.joint_status_text.append(f"<span style='color: #f47067;'>✗ Publish failed with exit code {exit_code}</span>")
        self.joint_status_text.append("")

    def send_goal(self):
        if not rclpy.ok():
            self.status_text.append("<span style='color: #c69026;'>⚠ ROS context invalid - cannot publish</span>")
            return
        try:
            pose = Pose()
            pose.position.x = self.x_input.value()
            pose.position.y = self.y_input.value()
            pose.position.z = self.z_input.value()
            pose.orientation.x = self.qx_input.value()
            pose.orientation.y = self.qy_input.value()
            pose.orientation.z = self.qz_input.value()
            pose.orientation.w = self.qw_input.value()
            self.goal_publisher.publish(pose)
            self.status_text.append(f"Sent goal: pos({pose.position.x:.2f}, {pose.position.y:.2f}, {pose.position.z:.2f}) orn({pose.orientation.x:.2f}, {pose.orientation.y:.2f}, {pose.orientation.z:.2f}, {pose.orientation.w:.2f})")
        except Exception as e:
            self.status_text.append(f"<span style='color: #f47067;'>❌ Failed to send goal: {e}</span>")
 
    def send_position_command(self):
        """Send position by launching node with selected position as argument"""
        position_name = self.position_dropdown.currentText()
 
        # Build command
        ros2_args = ['run', 'arm_control', 'position_sender_node', 
                     '--ros-args', '-r', '__ns:=/arm']
        
        # Build xterm command
        ros2_cmd = 'ros2 ' + ' '.join(ros2_args)
        xterm_args = ['-e', 'bash', '-c', ros2_cmd]
 
        # Display command in bold green
        cmd_str = 'xterm -e ' + ros2_cmd
        self.status_text.append(f"<b style='color: #57ab5a;'>→ {cmd_str}</b>")
        self.status_text.append("")  # Add newline after command
 
        # Launch the position_sender_node in xterm
        process = QProcess(self)
        process.setProcessChannelMode(QProcess.MergedChannels)
        process.readyReadStandardOutput.connect(lambda: self.handle_output(process))
 
        # Store process temporarily to prevent garbage collection
        process_key = f'position_cmd_{position_name}'
        process.finished.connect(lambda: self._cleanup_position_sender(process_key, position_name))
        process.start('xterm', xterm_args)
        self.process_map[process_key] = process
 
    def _cleanup_position_sender(self, process_key, position_name):
        """Clean up finished position sender process"""
        if process_key in self.process_map:
            del self.process_map[process_key]
        self.status_text.append(f"✓ Position '{position_name}' command completed")
 
    def emergency_stop(self):
        """Toggle emergency stop state (latched)."""
        self.set_emergency_stop_state(not self.emergency_stop_active, source="ui")
            
    def _on_emergency_stop_state(self, msg: Bool):
        """Sync UI with the latched /arm/emergency_stop state (even if published externally)."""
        self.set_emergency_stop_state(bool(msg.data), source="topic")

    def set_emergency_stop_state(self, active: bool, source: str = "ui"):
        """
        Centralized state setter.

        Behavior (matches old behavior):
        - Only publishes to /arm/emergency_stop when source == "ui"
        - Only publishes when the state actually changes
        - Topic callbacks only update UI (no re-publish), preventing feedback loops
        """
        # If no change, do nothing (prevents repeated publishes / UI churn)
        if active == getattr(self, "emergency_stop_active", False):
            return

        # Update internal state + UI
        self.emergency_stop_active = active
        self._update_emergency_stop_button_ui()
        # QApplication.processEvents()

        # If this came from the topic, do not publish or trigger side effects
        if source != "ui":
            return

        if not rclpy.ok():
            self.status_text.append("<span style='color: #c69026;'>⚠ ROS context invalid - cannot set emergency stop</span>")
            return

        # Publish only on user-triggered change
        try:
            stop_msg = Bool()
            stop_msg.data = active
            self.emergency_stop_publisher.publish(stop_msg)
        except Exception as e:
            self.status_text.append(f"<span style='color: #f47067;'>❌ Failed to publish /arm/emergency_stop: {e}</span>")
            return

        # User-triggered side-effects only
        try:
            if active:
                self.status_text.append("<span style='color: #c69026; font-weight: bold;'>⚠ EMERGENCY STOP - Published stop signal</span>")

                if self.current_goal_handle is not None:
                    self.current_goal_handle.cancel_goal_async()
                    self.status_text.append("<span style='color: #c69026;'>⚠ Canceling current trajectory goal...</span>")
                    self.current_goal_handle = None

                response = self._send_robot_command('stop')
                if response:
                    self.status_text.append("<span style='color: #c69026; font-weight: bold;'>⚠ Robot protective stop triggered</span>")

            else:
                self.status_text.append("<span style='color: #57ab5a; font-weight: bold;'>✓ EMERGENCY STOP RELEASED - Published release signal</span>")

                self._send_robot_command('close safety popup')
                response = self._send_robot_command('unlock protective stop')
                if response:
                    self.status_text.append("<span style='color: #57ab5a; font-weight: bold;'>✓ Robot protective stop released (requested)</span>")
                    
                    play_resp = self._send_robot_command('play')
                    if play_resp:
                        self.status_text.append("<span style='color: #57ab5a; font-weight: bold;'>✓ Robot program started (play)</span>")
                    
        except Exception as e:
            self.status_text.append(f"<span style='color: #f47067;'>❌ Error while applying emergency stop state: {e}</span>")


    def _update_emergency_stop_button_ui(self):
        """Update button label + color according to emergency stop state."""
        if not hasattr(self, "btn_emergency_stop"):
            return

        if self.emergency_stop_active:
            self.btn_emergency_stop.setText("EMERGENCY STOP ACTIVE (Click to Release)")
            self.btn_emergency_stop.setStyleSheet("background-color: #8b0000; color: white; font-weight: bold;")
            self.btn_emergency_stop.setChecked(True)
        else:
            self.btn_emergency_stop.setText("EMERGENCY STOP (Click to Activate)")
            self.btn_emergency_stop.setStyleSheet("background-color: red; color: white; font-weight: bold;")
            self.btn_emergency_stop.setChecked(False)
 
    def closeEvent(self, event):
        self.timer.stop()
        self.status_text.append("Shutting down and cleaning up processes...")
 
        # Terminate all launched processes
        for process_key, process in list(self.process_map.items()):
            if process.state() == QProcess.Running:
                # Get PID and kill entire process group
                pid = process.processId()
                self.status_text.append(f"Terminating {process_key} (PID: {pid})...")
 
                # Try graceful termination first
                process.terminate()
                if not process.waitForFinished(2000):
                    # Force kill if still running
                    process.kill()
                    process.waitForFinished(1000)
 
                # Kill entire process group to catch child processes
                if pid:
                    try:
                        subprocess.run(['pkill', '-9', '-P', str(pid)], timeout=1, stderr=subprocess.DEVNULL)
                    except:
                        pass

        # Kill any remaining ROS2 processes from this session
        self._cleanup_ros_children()
 
        # Close robot socket if connected
        if self.robot_socket:
            try:
                self.robot_socket.close()
            except:
                pass
 
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        event.accept()

    def _cleanup_ros_children(self):
        """Best-effort cleanup of ROS child processes launched by ros2 launch/run. Only called on window close."""
        patterns = [
            'rviz2',
            'ros2_control_node',
            'publisher_joint_trajectory_planned',
            'planner_node',
            'end_effector_pose_node',
            'position_sender_node',
            'sensors_orientation',
        ]
        for pat in patterns:
            try:
                subprocess.run(['pkill', '-9', '-f', pat], timeout=1, stderr=subprocess.DEVNULL)
            except:
                pass
 
    def _cleanup_ros_children_of_pid(self, parent_pid):
        """Kill specific ROS child processes that are descendants of the given parent PID."""
        if not parent_pid:
            return
 
        # Get all child PIDs recursively
        try:
            result = subprocess.run(['pgrep', '-P', str(parent_pid)], 
                                  capture_output=True, text=True, timeout=1)
            child_pids = result.stdout.strip().split('\n')
 
            for child_pid in child_pids:
                if child_pid:
                    # Recursively kill children's children
                    self._cleanup_ros_children_of_pid(int(child_pid))
                    # Kill this child
                    try:
                        subprocess.run(['kill', '-9', child_pid], timeout=1, stderr=subprocess.DEVNULL)
                    except:
                        pass
        except:
            pass
 
if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = RobotControlUI()
    window.show()
    sys.exit(app.exec_())