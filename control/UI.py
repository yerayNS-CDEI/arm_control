#!/usr/bin/env python3
import html
import json
import sys
import os
import shlex
import subprocess
import signal
import socket
import re
import time
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy
from PyQt5.QtWidgets import *
from PyQt5.QtCore import QTimer, QProcess, Qt
from PyQt5.QtGui import QFontMetrics, QIcon, QPixmap, QPalette, QColor
from PyQt5.QtWidgets import QApplication
from ament_index_python.packages import get_package_share_directory
from UI_utils.qtermwidget_wrapper import QTermWidget
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool, Float32MultiArray, Float64MultiArray
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from ur_msgs.msg import IOStates
from ur_msgs.srv import SetIO
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

def _detect_system_dark():
    """Return True if the GNOME/Ubuntu system colour-scheme is set to dark."""
    try:
        out = subprocess.run(
            ['gsettings', 'get', 'org.gnome.desktop.interface', 'color-scheme'],
            capture_output=True, text=True, timeout=2
        ).stdout
        return 'dark' in out.lower()
    except Exception:
        return False

def _make_dark_palette():
    p = QPalette()
    c = QColor
    p.setColor(QPalette.Window,          c('#1c2128'))
    p.setColor(QPalette.WindowText,      c('#cdd9e5'))
    p.setColor(QPalette.Base,            c('#2d333b'))
    p.setColor(QPalette.AlternateBase,   c('#22272e'))
    p.setColor(QPalette.ToolTipBase,     c('#2d333b'))
    p.setColor(QPalette.ToolTipText,     c('#cdd9e5'))
    p.setColor(QPalette.Text,            c('#cdd9e5'))
    p.setColor(QPalette.Button,          c('#2d333b'))
    p.setColor(QPalette.ButtonText,      c('#cdd9e5'))
    p.setColor(QPalette.BrightText,      c('#ffffff'))
    p.setColor(QPalette.Link,            c('#539bf5'))
    p.setColor(QPalette.Highlight,       c('#1f6feb'))
    p.setColor(QPalette.HighlightedText, c('#ffffff'))
    p.setColor(QPalette.Disabled, QPalette.WindowText, c('#768390'))
    p.setColor(QPalette.Disabled, QPalette.Text,       c('#768390'))
    p.setColor(QPalette.Disabled, QPalette.ButtonText, c('#768390'))
    p.setColor(QPalette.Disabled, QPalette.Base,       c('#22272e'))
    p.setColor(QPalette.Disabled, QPalette.Button,     c('#22272e'))
    return p

class RobotControlUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self._dark_theme = _detect_system_dark()
 
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
        self.emergency_stop_publisher = self.node.create_publisher(Bool, '/emergency_stop', qos)
        self.distance_sensor_publisher = self.node.create_publisher(Float32MultiArray, '/arm/distance_sensors', 10)
        self.emergency_stop_subscriber = self.node.create_subscription(Bool, '/emergency_stop', self._on_emergency_stop_state, qos)
        # Persistent publishers for emergency stop topics (keyed by topic name), created on demand
        self._emergency_stop_publishers = {}
        self.arm_joint_names = [
            'arm_shoulder_pan_joint',
            'arm_shoulder_lift_joint',
            'arm_elbow_joint',
            'arm_wrist_1_joint',
            'arm_wrist_2_joint',
            'arm_wrist_3_joint',
        ]
        self.arm_joint_limits = {
            'arm_shoulder_pan_joint': (-6.28, 6.28),
            'arm_shoulder_lift_joint': (-6.28, 6.28),
            'arm_elbow_joint': (-6.28, 6.28),
            'arm_wrist_1_joint': (-6.28, 6.28),
            'arm_wrist_2_joint': (-6.28, 6.28),
            'arm_wrist_3_joint': (-6.28, 6.28),
        }
        self.joint_state_subscriber = self.node.create_subscription(
            JointState,
            '/joint_states',
            lambda msg: self._on_joint_states(msg, '/joint_states'),
            10,
        )
        self.arm_joint_state_subscriber = self.node.create_subscription(
            JointState,
            '/arm/joint_states',
            lambda msg: self._on_joint_states(msg, '/arm/joint_states'),
            10,
        )
 
        # Track processes and their associated buttons
        self.process_map = {}
        self.button_map = {}
 
        # Store cleared status text for restore functionality
        self.cleared_status_backup = None
        self.base_cleared_status_backup = None
        self.gpr_cleared_status_backup = None
        self.fsm_cleared_status_backup = None

        # FSM processes
        self.fsm_launch_process = None
        self.fsm_node_process = None

        # FSM log buffer (html, plain_text, add_newline) for filter rebuilds
        self.fsm_log_entries = []
        self.fsm_log_backup = []
        
        # Store process list for kill functionality
        self.current_process_list = []
        self.ps_output_accumulator = ""
        
        # Emergency stop UI state
        self.emergency_stop_active = False
 
        # Robot dashboard connection
        self.robot_socket = None
        self.robot_host = '192.168.56.101'
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
 
        # Theme toggle button
        theme_bar = QHBoxLayout()
        theme_bar.addStretch()
        self.btn_theme_toggle = QPushButton("☀ Light Theme")
        self.btn_theme_toggle.setFixedWidth(120)
        self.btn_theme_toggle.clicked.connect(self._toggle_theme)
        theme_bar.addWidget(self.btn_theme_toggle)
        main_layout.addLayout(theme_bar)

        # Create tab widget for Arm Control and Base Control
        tabs = QTabWidget()
        main_layout.addWidget(tabs)

        self.tabs = tabs
        self.gpr_request_groups = self._build_gpr_request_groups()
        self.gpr_group_combos = {}
        self.gpr_request_processes = {}
        # Generic per-widget log buffers and filter inputs
        self.tab_log_entries: dict = {}   # id(widget) -> list of (html, plain_text, add_newline)
        self.tab_log_backups: dict = {}   # id(widget) -> backup list for clear/restore
        self.tab_filter_inputs: dict = {} # id(widget) -> QLineEdit
        # ===== ARM CONTROL TAB =====
        arm_tab = QWidget()
        arm_tab_layout = QVBoxLayout(arm_tab)
 
        # Simulation parameter selector (at top)
        arm_sim_param_layout = QHBoxLayout()
        arm_sim_param_layout.addWidget(QLabel("Simulation Mode:"))
        self.arm_sim_mode_combo = QComboBox()
        self.arm_sim_mode_combo.addItems(['false', 'true'])
        self.arm_sim_mode_combo.currentTextChanged.connect(self._update_init_box_state)
        self.arm_sim_mode_combo.currentTextChanged.connect(self._update_headless_visibility)
        self.arm_sim_mode_combo.currentTextChanged.connect(self._update_arm_planner_constraints)
        arm_sim_param_layout.addWidget(self.arm_sim_mode_combo)
        arm_sim_param_layout.addWidget(QLabel("Planner Backend:"))
        self.arm_planner_backend_combo = QComboBox()
        self.arm_planner_backend_combo.addItems(['moveit', 'legacy'])
        self.arm_planner_backend_combo.setCurrentText('legacy')
        self.arm_planner_backend_combo.currentTextChanged.connect(self._update_arm_planner_constraints)
        arm_sim_param_layout.addWidget(self.arm_planner_backend_combo)
        self.arm_moveit_pipeline_label = QLabel("MoveIt Pipeline:")
        arm_sim_param_layout.addWidget(self.arm_moveit_pipeline_label)
        self.arm_moveit_pipeline_combo = QComboBox()
        self.arm_moveit_pipeline_combo.addItems(['pilz_industrial_motion_planner', 'move_group'])
        self.arm_moveit_pipeline_combo.setCurrentText('pilz_industrial_motion_planner')
        self.arm_moveit_pipeline_combo.currentTextChanged.connect(
            self._update_moveit_option_visibility
        )
        arm_sim_param_layout.addWidget(self.arm_moveit_pipeline_combo)
        self.arm_moveit_planner_id_label = QLabel("Planner ID:")
        arm_sim_param_layout.addWidget(self.arm_moveit_planner_id_label)
        self.arm_moveit_planner_id_combo = QComboBox()
        self.arm_moveit_planner_id_combo.addItems(['PTP', 'LIN'])
        self.arm_moveit_planner_id_combo.setCurrentText('PTP')
        arm_sim_param_layout.addWidget(self.arm_moveit_planner_id_combo)
        self.arm_hybrid_sim_label = QLabel("URsim:")
        arm_sim_param_layout.addWidget(self.arm_hybrid_sim_label)
        self.arm_hybrid_sim_combo = QComboBox()
        self.arm_hybrid_sim_combo.addItems(['false', 'true'])
        self.arm_hybrid_sim_combo.setCurrentText('true')
        self.arm_hybrid_sim_combo.currentTextChanged.connect(self._update_init_box_state)
        self.arm_hybrid_sim_combo.currentTextChanged.connect(self._update_arm_planner_constraints)
        arm_sim_param_layout.addWidget(self.arm_hybrid_sim_combo)
        self.arm_moveit_status_label = QLabel("")
        self.arm_moveit_status_label.setStyleSheet("color: #0066cc; font-style: italic;")
        arm_sim_param_layout.addWidget(self.arm_moveit_status_label)
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
        self.btn_general_launch = QPushButton("Start Arm")
        self.btn_general_launch.clicked.connect(self.toggle_arm_launch)
        self.btn_general_launch.setToolTip("robot_ip=192.168.56.101 when URsim=true, else 192.168.1.102")
        control_layout.addWidget(self.btn_general_launch)
 
        # RQT Joint Controller button
        self.btn_rqt_controller = QPushButton("Launch RQT Joint Controller")
        self.btn_rqt_controller.clicked.connect(self.toggle_rqt_controller)
        self.btn_rqt_controller.setToolTip("ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller --force-discover --ros-args -r __ns:=/arm\n(Requires General Launch to be running)")
        control_layout.addWidget(self.btn_rqt_controller)
 
        # Dropdown for position selection
        control_layout.addWidget(QLabel("Select Position:"))
        position_sender_layout = QHBoxLayout()
        self.position_names = [
            'custom', 'folded', 'unfolded', 'up', 'down', 'front',
            'left', 'right', 'one', 'two', 'three', 'four', 'five',
            'six', 'p1', 'initial', 'under', 'under1', 'under2'
        ]
        self.position_dropdown = QComboBox()
        self.position_dropdown.addItems(self.position_names)
        position_sender_layout.addWidget(self.position_dropdown)
 
        btn_send_position = QPushButton("Send Position")
        btn_send_position.clicked.connect(self.send_position_command)
        btn_send_position.setToolTip("Uses /send_position for the Arm Control launch")
        position_sender_layout.addWidget(btn_send_position)
        control_layout.addLayout(position_sender_layout)
 
        # Reset Planner button
        self.btn_arm_reset_planner = QPushButton("Reset Planner")
        self.btn_arm_reset_planner.clicked.connect(self.reset_planner_arm)
        self.btn_arm_reset_planner.setToolTip('ros2 topic pub --once /planner/reset std_msgs/msg/Bool "{data: true}"')
        control_layout.addWidget(self.btn_arm_reset_planner)
 
        # List Controllers button
        btn_list_controllers = QPushButton("List Controllers")
        btn_list_controllers.clicked.connect(self.list_controllers)
        btn_list_controllers.setToolTip("List controllers for all detected controller_manager nodes")
        control_layout.addWidget(btn_list_controllers)
 
        # Emergency stop button
        goal_layout = QHBoxLayout()
 
        self.btn_emergency_stop = QPushButton("EMERGENCY STOP (Click to Activate)")
        self.btn_emergency_stop.setCheckable(True)
        self.btn_emergency_stop.setStyleSheet("background-color: red; color: white; font-weight: bold;")
        self.btn_emergency_stop.clicked.connect(lambda: self.emergency_stop(context='arm'))
        self.btn_emergency_stop.setToolTip("Toggle emergency stop on /emergency_stop and cancel trajectory goals when activating")
        goal_layout.addWidget(self.btn_emergency_stop)
        control_layout.addLayout(goal_layout)
 
        control_layout.addStretch()
 
        # Initialization Box (formerly Status, now first)
        init_box = QGroupBox("Initialization")
        init_layout = QVBoxLayout()
        init_box.setLayout(init_layout)
        self.init_box = init_box
        # Status Commands
        btn_send_all_status = QPushButton("Send All Status Commands")
        btn_send_all_status.clicked.connect(lambda: self.send_all_status_commands())
        btn_send_all_status.setToolTip("Send all status commands sequentially: robotmode, safetystatus, programState, running, get loaded program, is in remote control")
        init_layout.addWidget(btn_send_all_status)

        self.arm_status_indicators = self._build_status_indicators(init_layout)

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
        btn_send_control.clicked.connect(lambda: self.send_control_command())
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
        self.btn_arduino_sensors = QPushButton("Start Distance Sensors")
        self.btn_arduino_sensors.clicked.connect(self.toggle_arduino_sensors)
        self.btn_arduino_sensors.setToolTip("ros2 run arm_control arduino_sensors")
        sensors_layout.addWidget(self.btn_arduino_sensors)
 
        # Sensors orientation node button
        self.btn_align_ee_to_wall = QPushButton("Start Align EE to Wall")
        self.btn_align_ee_to_wall.clicked.connect(self.toggle_align_ee_to_wall)
        self.btn_align_ee_to_wall.setToolTip("ros2 run arm_control align_ee_to_wall")
        sensors_layout.addWidget(self.btn_align_ee_to_wall)
 
        sensors_layout.addStretch()
        boxes_layout.addWidget(sensors_box)
 
        # Live Joint Monitor and Status for Arm Control tab (side by side)
        arm_terminal_status_layout = QHBoxLayout()

        # Left side: Live joint position monitor (read-only)
        arm_joint_monitor = QGroupBox("Live Arm Joint Positions (rad)")
        arm_joint_monitor_layout = QVBoxLayout()
        arm_joint_monitor.setLayout(arm_joint_monitor_layout)
        self.arm_control_joint_sliders = []
        self.arm_control_joint_value_labels = []

        for joint_name in self.arm_joint_names:
            joint_row = QHBoxLayout()

            label = QLabel(f"{joint_name}:")
            label.setMinimumWidth(150)
            joint_row.addWidget(label)

            min_limit, max_limit = self.arm_joint_limits[joint_name]
            slider = QSlider(Qt.Horizontal)
            slider.setRange(int(min_limit * 100), int(max_limit * 100))
            slider.setValue(0)
            slider.setTickPosition(QSlider.TicksBelow)
            slider.setTickInterval(100)
            slider.setEnabled(False)
            joint_row.addWidget(slider)

            value_label = QLabel("0.000 rad")
            value_label.setMinimumWidth(95)
            joint_row.addWidget(value_label)

            self.arm_control_joint_sliders.append(slider)
            self.arm_control_joint_value_labels.append(value_label)
            arm_joint_monitor_layout.addLayout(joint_row)

        arm_joint_monitor_layout.addStretch()
        arm_terminal_status_layout.addWidget(arm_joint_monitor)
        
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
        status_header.addWidget(QLabel("Arm Control - Live Joints + Status"))
        status_header.addStretch()

        # Filter bar
        status_header.addWidget(QLabel("Filter:"))
        self.search_input = QLineEdit()
        self.search_input.setPlaceholderText("Filter output lines...")
        self.search_input.setMaximumWidth(200)
        self.search_input.textChanged.connect(lambda: self._log_apply_filter(self.status_text))
        status_header.addWidget(self.search_input)
        self.tab_filter_inputs[id(self.status_text)] = self.search_input

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
        self.sim_mode_combo.currentTextChanged.connect(self._update_headless_visibility)
        sim_param_layout.addWidget(self.sim_mode_combo)
        sim_param_layout.addWidget(QLabel("Controller Type:"))
        self.controller_type_combo = QComboBox()
        self.controller_type_combo.addItems(['omni', 'diff'])
        sim_param_layout.addWidget(self.controller_type_combo)
        self.base_headless_label = QLabel("Headless:")
        self.base_headless_combo = QComboBox()
        self.base_headless_combo.addItems(['false', 'true'])
        self.base_headless_combo.setCurrentText('true')
        sim_param_layout.addWidget(self.base_headless_label)
        sim_param_layout.addWidget(self.base_headless_combo)
        sim_param_layout.addStretch()
        base_tab_layout.insertLayout(0, sim_param_layout)
 
        # Mapping and Localization Box
        mapping_loc_box = QGroupBox("Mapping and Localization")
        mapping_loc_layout = QVBoxLayout()
        mapping_loc_box.setLayout(mapping_loc_layout)

        self.btn_launch_base_robot = QPushButton("Launch Base Robot")
        self.btn_launch_base_robot.clicked.connect(self.toggle_base_control_launch)
        self.btn_launch_base_robot.setToolTip(
            "ros2 launch navi_wall platform.launch.py sim:=<mode> mode:=base "
            "controller_type:=<type> odom_tf_from_controller:=true "
            "publish_controller_odom_tf:=true launch_rviz:=true "
            "headless:=<true/false>"
        )
        mapping_loc_layout.addWidget(self.btn_launch_base_robot)
 
        # Launch Mapping button
        self.btn_launch_mapping = QPushButton("Start Mapping")
        self.btn_launch_mapping.clicked.connect(lambda: self.toggle_mapping(controller_type_combo=self.controller_type_combo, headless_combo=self.base_headless_combo))
        self.btn_launch_mapping.setToolTip("ros2 launch navi_wall mapping_3d.launch.py sim:=<mode> mode:=base controller_type:=<type> headless:=<true/false>")
        mapping_loc_layout.addWidget(self.btn_launch_mapping)

        # Launch Localization button
        self.btn_launch_localization = QPushButton("Start Localization")
        self.btn_launch_localization.clicked.connect(lambda: self.toggle_localization(controller_type_combo=self.controller_type_combo, headless_combo=self.base_headless_combo))
        self.btn_launch_localization.setToolTip("ros2 launch navi_wall move_robot.launch.py sim:=<mode> mode:=base controller_type:=<type> headless:=<true/false>")
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
        self.btn_launch_nav2.clicked.connect(lambda: self.toggle_nav2(controller_type_combo=self.controller_type_combo))
        self.btn_launch_nav2.setToolTip("ros2 launch navi_wall navigation_launch.py use_sim_time:=<mode> controller_type:=<type>")
        nav_explore_layout.addWidget(self.btn_launch_nav2)
 
        # Launch Exploration button
        self.btn_launch_exploration = QPushButton("Launch Exploration(explore_lite)")
        self.btn_launch_exploration.clicked.connect(lambda: self.toggle_exploration())
        self.btn_launch_exploration.setToolTip("ros2 run navi_wall explore --ros-args --params-file arm_control/config/explore_params.yaml")
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
        btn_ps_ros.setToolTip("ps aux | grep -E 'ros2|robot'")
        troubleshooting_layout.addWidget(btn_ps_ros)
 
        # List Controllers button
        btn_list_base_controllers = QPushButton("List Controllers")
        btn_list_base_controllers.clicked.connect(self.run_list_base_controllers)
        btn_list_base_controllers.setToolTip("List controllers for all detected controller_manager nodes")
        troubleshooting_layout.addWidget(btn_list_base_controllers)
 
        # Launch RQT button
        self.btn_launch_rqt = QPushButton("Start RQT")
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

        # Filter bar
        base_status_header.addWidget(QLabel("Filter:"))
        self.base_search_input = QLineEdit()
        self.base_search_input.setPlaceholderText("Filter output lines...")
        self.base_search_input.setMaximumWidth(200)
        self.base_search_input.textChanged.connect(lambda: self._log_apply_filter(self.base_status_text))
        base_status_header.addWidget(self.base_search_input)
        self.tab_filter_inputs[id(self.base_status_text)] = self.base_search_input

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
        
        # Joint names and limits in the correct order for publishing
        joint_names = self.arm_joint_names
        joint_limits = self.arm_joint_limits

        # Create sliders and input fields for each joint
        self.joint_inputs = []
        self.joint_sliders = []
        for joint_name in joint_names:
            joint_row = QHBoxLayout()
            
            # Joint label
            label = QLabel(f"{joint_name}:")
            label.setMinimumWidth(150)
            joint_row.addWidget(label)
            
            # Get limits for this joint
            min_limit, max_limit = joint_limits[joint_name]
            
            # Slider (scale by 100 to handle 2 decimal places)
            slider = QSlider()
            slider.setOrientation(1)  # Horizontal
            slider.setRange(int(min_limit * 100), int(max_limit * 100))
            slider.setValue(0)
            slider.setTickPosition(QSlider.TicksBelow)
            slider.setTickInterval(100)
            joint_row.addWidget(slider)
            
            # Input field
            input_field = QDoubleSpinBox()
            input_field.setRange(min_limit, max_limit)
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
        btn_read_joints.setToolTip("Reads /arm/joint_states only for the Full Control hybrid launch, otherwise /joint_states")
        button_layout.addWidget(btn_read_joints)
        
        self.btn_publish_joints = QPushButton("Publish Joint Trajectory")
        self.btn_publish_joints.clicked.connect(self.publish_joint_trajectory)
        self.btn_publish_joints.setToolTip("Publishes to /arm/planned_trajectory only for the Full Control hybrid launch, otherwise /planned_trajectory")
        self.btn_publish_joints.setEnabled(False)  # Disabled until positions are read
        button_layout.addWidget(self.btn_publish_joints)

        # Reset Planner button for Joint Control tab
        self.btn_joint_reset_planner = QPushButton("Reset Planner")
        self.btn_joint_reset_planner.clicked.connect(self.reset_planner_joint)
        self.btn_joint_reset_planner.setToolTip('ros2 topic pub --once /planner/reset std_msgs/msg/Bool "{data: true}"')
        button_layout.addWidget(self.btn_joint_reset_planner)
        
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
        # ===== FULL CONTROL TAB =====
        full_control_tab = QWidget()
        full_control_tab_layout = QVBoxLayout(full_control_tab)

        # Simulation parameter selector (at top)
        full_control_sim_param_layout = QHBoxLayout()
        full_control_sim_param_layout.addWidget(QLabel("Simulation Mode:"))
        self.full_control_sim_mode_combo = QComboBox()
        self.full_control_sim_mode_combo.addItems(['false', 'true'])
        self.full_control_sim_mode_combo.currentTextChanged.connect(self._update_full_control_sim_mode)
        full_control_sim_param_layout.addWidget(self.full_control_sim_mode_combo)
        full_control_sim_param_layout.addWidget(QLabel("Controller Type:"))
        self.full_control_controller_type_combo = QComboBox()
        self.full_control_controller_type_combo.addItems(['omni', 'diff'])
        full_control_sim_param_layout.addWidget(self.full_control_controller_type_combo)
        full_control_sim_param_layout.addWidget(QLabel("Planner Backend:"))
        self.full_control_planner_backend_combo = QComboBox()
        self.full_control_planner_backend_combo.addItems(['moveit', 'legacy'])
        self.full_control_planner_backend_combo.setCurrentText('legacy')
        self.full_control_planner_backend_combo.currentTextChanged.connect(self._update_full_control_planner_constraints)
        full_control_sim_param_layout.addWidget(self.full_control_planner_backend_combo)
        self.full_control_moveit_pipeline_label = QLabel("MoveIt Pipeline:")
        full_control_sim_param_layout.addWidget(self.full_control_moveit_pipeline_label)
        self.full_control_moveit_pipeline_combo = QComboBox()
        self.full_control_moveit_pipeline_combo.addItems(['pilz_industrial_motion_planner', 'move_group'])
        self.full_control_moveit_pipeline_combo.setCurrentText('pilz_industrial_motion_planner')
        self.full_control_moveit_pipeline_combo.currentTextChanged.connect(
            self._update_moveit_option_visibility
        )
        full_control_sim_param_layout.addWidget(self.full_control_moveit_pipeline_combo)
        self.full_control_moveit_planner_id_label = QLabel("Planner ID:")
        full_control_sim_param_layout.addWidget(self.full_control_moveit_planner_id_label)
        self.full_control_moveit_planner_id_combo = QComboBox()
        self.full_control_moveit_planner_id_combo.addItems(['PTP', 'LIN'])
        self.full_control_moveit_planner_id_combo.setCurrentText('PTP')
        full_control_sim_param_layout.addWidget(self.full_control_moveit_planner_id_combo)
        self.full_control_hybrid_sim_label = QLabel("Hybrid Sim:")
        self.full_control_hybrid_sim_combo = QComboBox()
        self.full_control_hybrid_sim_combo.addItems(['false', 'true'])
        self.full_control_hybrid_sim_combo.setCurrentText('false')
        self.full_control_hybrid_sim_combo.currentTextChanged.connect(self._on_full_control_hybrid_changed)
        full_control_sim_param_layout.addWidget(self.full_control_hybrid_sim_label)
        full_control_sim_param_layout.addWidget(self.full_control_hybrid_sim_combo)
        self.full_control_headless_label = QLabel("Headless:")
        self.full_control_headless_combo = QComboBox()
        self.full_control_headless_combo.addItems(['false', 'true'])
        self.full_control_headless_combo.setCurrentText('true')
        self.full_control_headless_combo.currentTextChanged.connect(
            self._update_full_control_launch_tooltip
        )
        full_control_sim_param_layout.addWidget(self.full_control_headless_label)
        full_control_sim_param_layout.addWidget(self.full_control_headless_combo)
        full_control_sim_param_layout.addStretch()
        full_control_tab_layout.addLayout(full_control_sim_param_layout)

        # Horizontal layout for control boxes side by side
        full_control_boxes_layout = QHBoxLayout()
        full_control_tab_layout.addLayout(full_control_boxes_layout)

        # Initialization Box (create new comboboxes for Full Control)
        full_control_init_box = QGroupBox("Robot Initialization")
        full_control_init_layout = QVBoxLayout()
        full_control_init_box.setLayout(full_control_init_layout)

        btn_full_control_send_all_status = QPushButton("Send All Status Commands")
        btn_full_control_send_all_status.clicked.connect(
            lambda: self.send_all_full_control_status_commands()
        )
        full_control_init_layout.addWidget(btn_full_control_send_all_status)

        self.full_status_indicators = self._build_status_indicators(full_control_init_layout)

        # Control commands
        full_control_init_layout.addWidget(QLabel("\nControl Commands:"))
        self.full_control_control_cmd_combo = QComboBox()
        self.full_control_control_cmd_combo.addItems([
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
        full_control_init_layout.addWidget(self.full_control_control_cmd_combo)
        
        btn_full_control_send_control = QPushButton("Send Control Command")
        btn_full_control_send_control.clicked.connect(
            lambda: self.send_full_control_control_command()
        )
        full_control_init_layout.addWidget(btn_full_control_send_control)

        full_control_init_layout.addStretch()
        full_control_boxes_layout.addWidget(full_control_init_box)
        self.full_control_init_box = full_control_init_box

        # Mapping, Localization, Navigation, Exploration Box
        full_control_mapping_box = QGroupBox("Mapping, Localization, Navigation, Exploration")
        full_control_mapping_layout = QVBoxLayout()
        full_control_mapping_box.setLayout(full_control_mapping_layout)

        self.btn_full_control_launch = QPushButton("Launch Full Robot")
        self.btn_full_control_launch.clicked.connect(self.toggle_full_control_launch)
        self.btn_full_control_launch.setToolTip(
            "Uses `ros2 launch navi_wall hybrid_simulation.launch.py` when "
            "`hybrid_sim=true`, otherwise `ros2 launch navi_wall "
            "oliwall_mobile_manipulator.launch.py`."
        )
        full_control_mapping_layout.addWidget(self.btn_full_control_launch)

        self.btn_full_control_mapping = QPushButton("Start Mapping")
        self.btn_full_control_mapping.clicked.connect(
            lambda: self.toggle_mapping(
                mode='full',
                button=self.btn_full_control_mapping,
                sim_combo=self.full_control_sim_mode_combo,
                controller_type_combo=self.full_control_controller_type_combo,
                headless_combo=self.full_control_headless_combo,
                hybrid_sim_combo=self.full_control_hybrid_sim_combo,
            )
        )
        self.btn_full_control_mapping.setToolTip("ros2 launch navi_wall mapping_3d.launch.py sim:=<mode> mode:=full controller_type:=<type> hybrid_sim:=<true/false> headless:=<true/false>")
        full_control_mapping_layout.addWidget(self.btn_full_control_mapping)

        self.btn_full_control_localization = QPushButton("Start Localization")
        self.btn_full_control_localization.clicked.connect(
            lambda: self.toggle_localization(
                mode='full',
                button=self.btn_full_control_localization,
                sim_combo=self.full_control_sim_mode_combo,
                controller_type_combo=self.full_control_controller_type_combo,
                headless_combo=self.full_control_headless_combo,
                hybrid_sim_combo=self.full_control_hybrid_sim_combo,
            )
        )
        self.btn_full_control_localization.setToolTip("ros2 launch navi_wall move_robot.launch.py sim:=<mode> mode:=full controller_type:=<type> hybrid_sim:=<true/false> planner_backend:=<moveit|legacy> [moveit_planning_pipeline:=<...> moveit_pose_planner_id:=<...>] headless:=<true/false>")
        full_control_mapping_layout.addWidget(self.btn_full_control_localization)

        self.btn_full_control_nav2 = QPushButton("Launch Nav2")
        self.btn_full_control_nav2.clicked.connect(lambda: self.toggle_nav2(mode='full', button=self.btn_full_control_nav2, sim_combo=self.full_control_sim_mode_combo, controller_type_combo=self.full_control_controller_type_combo))
        self.btn_full_control_nav2.setToolTip("ros2 launch navi_wall navigation_launch.py use_sim_time:= controller_type:=<type>")
        full_control_mapping_layout.addWidget(self.btn_full_control_nav2)

        self.btn_full_control_exploration = QPushButton("Launch Exploration (explore_lite)")
        self.btn_full_control_exploration.clicked.connect(lambda: self.toggle_exploration(mode='full', button=self.btn_full_control_exploration, sim_combo=self.full_control_sim_mode_combo))
        self.btn_full_control_exploration.setToolTip("ros2 run navi_wall explore --ros-args --params-file config/explore_params.yaml")
        full_control_mapping_layout.addWidget(self.btn_full_control_exploration)

        full_control_position_separator = QFrame()
        full_control_position_separator.setFrameShape(QFrame.HLine)
        full_control_position_separator.setFrameShadow(QFrame.Sunken)
        full_control_mapping_layout.addWidget(full_control_position_separator)

        full_control_mapping_layout.addWidget(QLabel("Select Position:"))
        full_control_position_layout = QHBoxLayout()
        self.full_control_position_dropdown = QComboBox()
        self.full_control_position_dropdown.addItems(self.position_names)
        full_control_position_layout.addWidget(self.full_control_position_dropdown)

        btn_full_control_send_position = QPushButton("Send Position")
        btn_full_control_send_position.clicked.connect(self.send_full_control_position_command)
        btn_full_control_send_position.setToolTip("Uses /arm/send_position when hybrid_sim=true, otherwise /send_position")
        full_control_position_layout.addWidget(btn_full_control_send_position)
        full_control_mapping_layout.addLayout(full_control_position_layout)

        # Reset Planner button
        self.btn_full_control_reset_planner = QPushButton("Reset Planner")
        self.btn_full_control_reset_planner.clicked.connect(self.reset_planner)
        self.btn_full_control_reset_planner.setToolTip('ros2 topic pub --once /planner/reset std_msgs/msg/Bool "{data: true}"')
        full_control_mapping_layout.addWidget(self.btn_full_control_reset_planner)

        # Emergency stop button for Full Control tab
        full_control_mapping_layout.addWidget(QLabel(""))  # Spacer
        self.btn_full_control_emergency_stop = QPushButton("EMERGENCY STOP (Click to Activate)")
        self.btn_full_control_emergency_stop.setCheckable(True)
        self.btn_full_control_emergency_stop.setStyleSheet("background-color: red; color: white; font-weight: bold;")
        self.btn_full_control_emergency_stop.clicked.connect(lambda: self.emergency_stop(context='full'))
        self.btn_full_control_emergency_stop.setToolTip("Toggle emergency stop and cancel trajectory goals when activating\nUses /arm/emergency_stop when simulation=true AND hybrid_sim=true, otherwise /emergency_stop")
        full_control_mapping_layout.addWidget(self.btn_full_control_emergency_stop)

        full_control_mapping_layout.addStretch()
        full_control_boxes_layout.addWidget(full_control_mapping_box)

        # Sensors Box (Full Control)
        full_control_sensors_box = QGroupBox("Sensors")
        full_control_sensors_layout = QVBoxLayout()
        full_control_sensors_box.setLayout(full_control_sensors_layout)

        # Set background image for sensors box using a label
        try:
            pkg_share = get_package_share_directory('arm_control')
            bg_image_path = os.path.join(pkg_share, 'resource', 'oliwall.png')
        except Exception:
            # Fallback to source directory if package not found
            bg_image_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'resource', 'oliwall.png')
        
        if os.path.exists(bg_image_path):
            # Create a label to hold the background image
            self.full_control_sensors_bg_label = QLabel(full_control_sensors_box)
            pixmap = QPixmap(bg_image_path)
            # Scale pixmap to fit within a reasonable size while maintaining aspect ratio
            scaled_pixmap = pixmap.scaled(250, 200, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            self.full_control_sensors_bg_label.setPixmap(scaled_pixmap)
            self.full_control_sensors_bg_label.setAlignment(Qt.AlignCenter)
            self.full_control_sensors_bg_label.setScaledContents(False)
            full_control_sensors_layout.addWidget(self.full_control_sensors_bg_label)

        self.btn_full_control_align_ee = QPushButton("Start Align EE to Wall")
        self.btn_full_control_align_ee.clicked.connect(
            lambda: self.toggle_full_control_align_ee()
        )
        self.btn_full_control_align_ee.setToolTip("ros2 run arm_control align_ee_to_wall")
        full_control_sensors_layout.addWidget(self.btn_full_control_align_ee)

        full_control_sensors_layout.addStretch()
        full_control_boxes_layout.addWidget(full_control_sensors_box)

        # Troubleshooting Box (Full Control)
        full_control_troubleshooting_box = QGroupBox("Troubleshooting")
        full_control_troubleshooting_layout = QVBoxLayout()
        full_control_troubleshooting_box.setLayout(full_control_troubleshooting_layout)

        self.btn_full_control_view_map = QPushButton("View map")
        self.btn_full_control_view_map.clicked.connect(
            lambda: self.toggle_view_map(mode='full', button=self.btn_full_control_view_map)
        )
        self.btn_full_control_view_map.setToolTip("rtabmap-databaseViewer rtabmap.db")
        full_control_troubleshooting_layout.addWidget(self.btn_full_control_view_map)

        self.btn_full_control_list_controllers = QPushButton("List Controllers")
        self.btn_full_control_list_controllers.clicked.connect(self.run_list_full_controllers)
        self.btn_full_control_list_controllers.setToolTip("List controllers for all detected controller_manager nodes")
        full_control_troubleshooting_layout.addWidget(self.btn_full_control_list_controllers)

        self.btn_full_control_rqt = QPushButton("Start RQT")
        self.btn_full_control_rqt.clicked.connect(
            lambda: self.toggle_rqt(mode='full', button=self.btn_full_control_rqt)
        )
        self.btn_full_control_rqt.setToolTip("rqt")
        full_control_troubleshooting_layout.addWidget(self.btn_full_control_rqt)

        # List ROS2 Processes button
        btn_full_control_ps_ros = QPushButton("List ROS2 Processes")
        btn_full_control_ps_ros.clicked.connect(self.run_full_control_ps_ros)
        btn_full_control_ps_ros.setToolTip("ps aux | grep -E 'ros2|robot'")
        full_control_troubleshooting_layout.addWidget(btn_full_control_ps_ros)

        # ROS2 Topics Section
        full_control_troubleshooting_layout.addWidget(QLabel("\nROS2 Topics:"))
        
        # Topic selector with refresh button
        topic_selector_layout = QHBoxLayout()
        self.topics_combo = QComboBox()
        self.topics_combo.setEditable(True)  # Enable search/filtering
        self.topics_combo.setMinimumWidth(200)
        topic_selector_layout.addWidget(self.topics_combo)
        
        btn_refresh_topics = QPushButton("Refresh")
        btn_refresh_topics.clicked.connect(self.refresh_topics_list)
        btn_refresh_topics.setToolTip("Refresh available ROS2 topics")
        btn_refresh_topics.setMaximumWidth(80)
        topic_selector_layout.addWidget(btn_refresh_topics)
        full_control_troubleshooting_layout.addLayout(topic_selector_layout)
        
        # Topic action buttons
        topic_actions_layout = QHBoxLayout()
        
        btn_topic_bw = QPushButton("Bandwidth")
        btn_topic_bw.clicked.connect(self.check_topic_bandwidth)
        btn_topic_bw.setToolTip("Check topic bandwidth (ros2 topic bw)")
        topic_actions_layout.addWidget(btn_topic_bw)
        
        btn_topic_hz = QPushButton("Frequency")
        btn_topic_hz.clicked.connect(self.check_topic_frequency)
        btn_topic_hz.setToolTip("Check topic frequency (ros2 topic hz)")
        topic_actions_layout.addWidget(btn_topic_hz)
        
        btn_topic_echo = QPushButton("Echo Once")
        btn_topic_echo.clicked.connect(self.echo_topic_once)
        btn_topic_echo.setToolTip("Echo topic once (ros2 topic echo --once)")
        topic_actions_layout.addWidget(btn_topic_echo)
        
        full_control_troubleshooting_layout.addLayout(topic_actions_layout)
        
        # Topic info display area
        self.topic_info_display = QTextEdit()
        self.topic_info_display.setReadOnly(True)
        self.topic_info_display.setMaximumHeight(60)
        self.topic_info_display.setPlaceholderText("Topic bandwidth and frequency info will appear here...")
        self.topic_info_display.setStyleSheet("background-color: #f6f8fa; color: #1f2328; border: 1px solid #d0d7de; font-family: 'Courier New', monospace; padding: 4px;")
        full_control_troubleshooting_layout.addWidget(self.topic_info_display)

        # Process Kill Section
        full_control_troubleshooting_layout.addWidget(QLabel("\nProcess Management:"))
        
        # Process selector with kill button
        process_selector_layout = QHBoxLayout()
        self.process_combo = QComboBox()
        self.process_combo.setEditable(False)
        self.process_combo.setMinimumWidth(150)
        self.process_combo.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.process_combo.setPlaceholderText("Select process to kill...")
        process_selector_layout.addWidget(self.process_combo, 1)  # Stretch factor of 1
        
        btn_kill_process = QPushButton("Kill Process")
        btn_kill_process.clicked.connect(self.kill_selected_process)
        btn_kill_process.setToolTip("Kill the selected process using kill -9")
        btn_kill_process.setMaximumWidth(120)
        btn_kill_process.setStyleSheet("background-color: #d73a49; color: white; font-weight: bold;")
        process_selector_layout.addWidget(btn_kill_process, 0)  # No stretch
        
        btn_kill_all = QPushButton("Kill All")
        btn_kill_all.clicked.connect(self.kill_all_processes)
        btn_kill_all.setToolTip("Kill all detected processes except UI and ros2 daemon")
        btn_kill_all.setMaximumWidth(120)
        btn_kill_all.setStyleSheet("background-color: #8B0000; color: white; font-weight: bold;")
        process_selector_layout.addWidget(btn_kill_all, 0)  # No stretch
        
        full_control_troubleshooting_layout.addLayout(process_selector_layout)

        full_control_troubleshooting_layout.addStretch()
        full_control_boxes_layout.addWidget(full_control_troubleshooting_box)

        # Live Joint Monitor and Status for Full Control tab (side by side)
        full_control_terminal_status_layout = QHBoxLayout()

        # Left side: Live joint position monitor (read-only)
        full_control_joint_monitor = QGroupBox("Live Arm Joint Positions (rad)")
        full_control_joint_monitor_layout = QVBoxLayout()
        full_control_joint_monitor.setLayout(full_control_joint_monitor_layout)
        self.full_control_joint_sliders = []
        self.full_control_joint_value_labels = []

        for joint_name in self.arm_joint_names:
            joint_row = QHBoxLayout()

            label = QLabel(f"{joint_name}:")
            label.setMinimumWidth(150)
            joint_row.addWidget(label)

            min_limit, max_limit = self.arm_joint_limits[joint_name]
            slider = QSlider(Qt.Horizontal)
            slider.setRange(int(min_limit * 100), int(max_limit * 100))
            slider.setValue(0)
            slider.setTickPosition(QSlider.TicksBelow)
            slider.setTickInterval(100)
            slider.setEnabled(False)
            joint_row.addWidget(slider)

            value_label = QLabel("0.000 rad")
            value_label.setMinimumWidth(95)
            joint_row.addWidget(value_label)

            self.full_control_joint_sliders.append(slider)
            self.full_control_joint_value_labels.append(value_label)
            full_control_joint_monitor_layout.addLayout(joint_row)

        full_control_joint_monitor_layout.addStretch()
        full_control_terminal_status_layout.addWidget(full_control_joint_monitor)

        # Right side: NEW Status display for Full Control
        self.full_control_status_text = QTextEdit()
        self.full_control_status_text.setReadOnly(True)
        self.full_control_status_text.setAcceptRichText(True)
        self.full_control_status_text.setStyleSheet("background-color: #22272e; color: #adbac7; border: 1px solid #444c56; font-family: 'Courier New', monospace; white-space: pre;")

        # Set tab stops to 8 characters (standard terminal width)
        from PyQt5.QtGui import QFontMetrics
        font_metrics = QFontMetrics(self.full_control_status_text.font())
        tab_width = font_metrics.horizontalAdvance(' ') * 8
        self.full_control_status_text.setTabStopDistance(tab_width)
        full_control_terminal_status_layout.addWidget(self.full_control_status_text)

        # Status header with search functionality
        full_control_status_header = QHBoxLayout()
        full_control_status_header.addWidget(QLabel("Full Control - Live Joints + Status"))
        full_control_status_header.addStretch()

        # Filter bar for Full Control
        full_control_status_header.addWidget(QLabel("Filter:"))
        self.full_control_search_input = QLineEdit()
        self.full_control_search_input.setPlaceholderText("Filter output lines...")
        self.full_control_search_input.setMaximumWidth(200)
        self.full_control_search_input.textChanged.connect(lambda: self._log_apply_filter(self.full_control_status_text))
        full_control_status_header.addWidget(self.full_control_search_input)
        self.tab_filter_inputs[id(self.full_control_status_text)] = self.full_control_search_input

        self.btn_restore_full_control_status = QPushButton("Restore")
        self.btn_restore_full_control_status.clicked.connect(self.restore_full_control_status)
        self.btn_restore_full_control_status.setMaximumWidth(80)
        self.btn_restore_full_control_status.setVisible(False)
        full_control_status_header.addWidget(self.btn_restore_full_control_status)

        btn_clear_full_control_status = QPushButton("Clear")
        btn_clear_full_control_status.clicked.connect(self.clear_full_control_status)
        btn_clear_full_control_status.setMaximumWidth(80)
        full_control_status_header.addWidget(btn_clear_full_control_status)

        full_control_tab_layout.addLayout(full_control_status_header)
        full_control_tab_layout.addLayout(full_control_terminal_status_layout)
        # Add Full Control tab after Joint Control
        tabs.addTab(full_control_tab, "Full Control")
        tabs.addTab(self._create_gpr_api_test_tab(), "GPR API Test")
        tabs.addTab(self._create_fsm_tab(), "FSM")

        # Connect tab change signal to check joint states when Joint Control tab is activated
        tabs.currentChanged.connect(lambda index: self._on_tab_changed(index, tabs))
        
        # Timer for ROS spinning
        self.timer = QTimer()
        self.timer.timeout.connect(self._spin_ros)
        self.timer.start(100)  # 10 Hz
        
        # Initial UI update for consistent visuals
        self._update_emergency_stop_button_ui()
        self.BASE_TAB_INDEX = 0
        self.ARM_TAB_INDEX = 1
        self.JOINT_TAB_INDEX = 2
        self.FULL_CONTROL_TAB_INDEX = 3
        self._active_planner_context = 'arm'
        self._update_init_box_state()
        self._update_full_control_init_box_state()
        self._update_headless_visibility()
        self._update_arm_planner_constraints()
        self._update_full_control_planner_constraints()
        
        # Initial topics list refresh
        QTimer.singleShot(1000, self.refresh_topics_list)  # Delay 1s to ensure ROS is ready

        self._apply_theme()

    def _toggle_theme(self):
        self._dark_theme = not self._dark_theme
        self._apply_theme()

    def _apply_theme(self):
        dark = self._dark_theme
        app = QApplication.instance()
        if dark:
            app.setPalette(_make_dark_palette())
            log_style = (
                "background-color: #22272e; color: #adbac7; border: 1px solid #444c56; "
                "font-family: 'Courier New', monospace;"
            )
            info_style = (
                "background-color: #22272e; color: #adbac7; border: 1px solid #444c56; "
                "font-family: 'Courier New', monospace; padding: 4px;"
            )
        else:
            app.setPalette(app.style().standardPalette())
            log_style = (
                "background-color: #f0f0f0; color: #1f2328; border: 1px solid #d0d7de; "
                "font-family: 'Courier New', monospace;"
            )
            info_style = (
                "background-color: #f0f0f0; color: #1f2328; border: 1px solid #d0d7de; "
                "font-family: 'Courier New', monospace; padding: 4px;"
            )

        for attr in ('status_text', 'base_status_text', 'joint_status_text',
                     'full_control_status_text', 'gpr_status_text'):
            if hasattr(self, attr):
                getattr(self, attr).setStyleSheet(log_style)

        if hasattr(self, 'topic_info_display'):
            self.topic_info_display.setStyleSheet(info_style)

        if hasattr(self, 'btn_theme_toggle'):
            self.btn_theme_toggle.setText("☀ Light Theme" if dark else "☾ Dark Theme")

    def _update_full_control_sim_mode(self):
        """Refresh Full Control state when its simulation mode changes."""
        self._update_full_control_planner_constraints()

    def _on_full_control_hybrid_changed(self):
        """Refresh dependent UI elements when Full Control hybrid_sim changes."""
        self._update_full_control_init_box_state()
        self._update_headless_visibility()
        self._update_full_control_launch_tooltip()

    def _is_robot_bringup_process(self, process_key):
        """Return whether a process key belongs to a robot bringup launch."""
        return process_key in {'mobile_platform', 'full_mobile_manipulator'}

    def _is_base_tab_state_process(self, process_key):
        """Return whether a process should lock the Base Control tab context."""
        return process_key in {'mobile_platform', 'mapping', 'localization', 'nav2', 'exploration'}

    def _get_base_process_start_text(self, process_key, name):
        """Return the idle button label for base/full-control launch buttons."""
        if process_key == 'full_mobile_manipulator':
            return 'Launch Full Robot'
        if process_key == 'mobile_platform':
            return 'Launch Base Robot'
        return f"Start {name}"

    def _get_base_process_stop_text(self, process_key, name):
        """Return the running button label for base/full-control launch buttons."""
        if process_key == 'full_mobile_manipulator':
            return 'Stop Full Robot'
        if process_key == 'mobile_platform':
            return 'Stop Base Robot'
        return f"Stop {name}"

    def _sync_full_control_hybrid_sim_state(self):
        """Apply Full Control hybrid_sim constraints and return the effective value."""
        sim_mode = (
            self.full_control_sim_mode_combo.currentText()
            if hasattr(self, 'full_control_sim_mode_combo')
            else 'false'
        )
        planner_backend = self._get_planner_backend(context='full')

        if sim_mode != 'true':
            desired_hybrid_sim = 'false'
            should_lock_hybrid_sim = True
        elif hasattr(self, 'full_control_hybrid_sim_combo'):
            # When sim=true: user is free to choose regardless of planner backend
            desired_hybrid_sim = self.full_control_hybrid_sim_combo.currentText()
            should_lock_hybrid_sim = False
        else:
            desired_hybrid_sim = 'false'
            should_lock_hybrid_sim = False

        if hasattr(self, 'full_control_hybrid_sim_combo'):
            previous_signal_state = self.full_control_hybrid_sim_combo.blockSignals(True)
            if should_lock_hybrid_sim:
                self.full_control_hybrid_sim_combo.setCurrentText(desired_hybrid_sim)
            self.full_control_hybrid_sim_combo.setEnabled(not should_lock_hybrid_sim)
            self.full_control_hybrid_sim_combo.blockSignals(previous_signal_state)

        return desired_hybrid_sim

    def _get_full_control_launch_file(self, hybrid_sim=None):
        """Pick the Full Control launch file from the effective hybrid_sim value."""
        if hybrid_sim is None:
            hybrid_sim = self._sync_full_control_hybrid_sim_state()
        if hybrid_sim == 'true':
            return 'hybrid_simulation.launch.py'
        return 'oliwall_mobile_manipulator.launch.py'

    def _update_full_control_launch_tooltip(self):
        """Keep the Full Control launch tooltip aligned with the selected launch file."""
        if not hasattr(self, 'btn_full_control_launch'):
            return

        hybrid_sim = self._sync_full_control_hybrid_sim_state()
        launch_file = self._get_full_control_launch_file(hybrid_sim=hybrid_sim)
        headless = (
            self.full_control_headless_combo.currentText()
            if hasattr(self, 'full_control_headless_combo')
            else 'false'
        )

        if launch_file == 'hybrid_simulation.launch.py':
            tooltip = (
                "ros2 launch navi_wall hybrid_simulation.launch.py "
                "mode:=full robot_ip:=<auto> controller_type:=<type> "
                f"planner_backend:=<moveit|legacy> headless:={headless} "
                "[moveit_planning_pipeline:=<...> moveit_pose_planner_id:=<...>]"
            )
        else:
            tooltip = (
                "ros2 launch navi_wall oliwall_mobile_manipulator.launch.py "
                "mode:=full sim:=<mode> hybrid_sim:=<true/false> robot_ip:=<auto> "
                f"controller_type:=<type> planner_backend:=<moveit|legacy> headless:={headless} "
                "[moveit_planning_pipeline:=<...> moveit_pose_planner_id:=<...>]"
            )

        self.btn_full_control_launch.setToolTip(tooltip)

    def _update_arm_planner_constraints(self):
        """Lock/unlock URSim selector based on Arm tab planner backend selection and simulation mode."""
        planner = self._get_planner_backend(context='arm')
        if hasattr(self, 'arm_hybrid_sim_combo'):
            if planner == 'moveit':
                # Get current simulation mode
                sim_mode = self.arm_sim_mode_combo.currentText()
                if sim_mode == 'true':
                    # Allow user to freely choose URsim or Gazebo when simulating with MoveIt
                    self.arm_hybrid_sim_combo.setEnabled(True)
                    if hasattr(self, 'arm_moveit_status_label'):
                        self.arm_moveit_status_label.setVisible(False)
                else:
                    # Real robot: lock URsim to false
                    self.arm_hybrid_sim_combo.setCurrentText('false')
                    self.arm_hybrid_sim_combo.setEnabled(False)
                    if hasattr(self, 'arm_moveit_status_label'):
                        self.arm_moveit_status_label.setText("Running moveit in Real Robot")
                        self.arm_moveit_status_label.setVisible(True)
            else:
                self.arm_hybrid_sim_combo.setEnabled(True)
                # Hide status label when not using MoveIt
                if hasattr(self, 'arm_moveit_status_label'):
                    self.arm_moveit_status_label.setVisible(False)

        # Disable Reset Planner button when backend is moveit
        if hasattr(self, 'btn_arm_reset_planner'):
            self.btn_arm_reset_planner.setEnabled(planner != 'moveit')

        # Update Joint Control tab state based on the active planner tab
        self._update_joint_control_tab_state()

        self._update_moveit_option_visibility()
        self._update_headless_visibility()
        self._update_init_box_state()

    def _update_full_control_planner_constraints(self):
        """Keep Full Control hybrid_sim aligned with simulation and refresh planner UI."""
        planner = self._get_planner_backend(context='full')
        self._sync_full_control_hybrid_sim_state()

        # Disable Reset Planner button when backend is moveit
        if hasattr(self, 'btn_full_control_reset_planner'):
            self.btn_full_control_reset_planner.setEnabled(planner != 'moveit')

        # Update Joint Control tab state based on the active planner tab
        self._update_joint_control_tab_state()

        self._update_moveit_option_visibility()
        self._update_full_control_init_box_state()
        self._update_headless_visibility()
        self._update_full_control_launch_tooltip()

    def _update_moveit_option_visibility(self):
        """Show MoveIt pipeline/planner selectors only when they are relevant."""
        arm_backend_is_moveit = self._get_planner_backend(context='arm') == 'moveit'
        arm_pipeline_is_pilz = (
            hasattr(self, 'arm_moveit_pipeline_combo')
            and self.arm_moveit_pipeline_combo.currentText() == 'pilz_industrial_motion_planner'
        )
        if hasattr(self, 'arm_moveit_pipeline_label'):
            self.arm_moveit_pipeline_label.setVisible(arm_backend_is_moveit)
        if hasattr(self, 'arm_moveit_pipeline_combo'):
            self.arm_moveit_pipeline_combo.setVisible(arm_backend_is_moveit)
        if hasattr(self, 'arm_moveit_planner_id_label'):
            self.arm_moveit_planner_id_label.setVisible(
                arm_backend_is_moveit and arm_pipeline_is_pilz
            )
        if hasattr(self, 'arm_moveit_planner_id_combo'):
            self.arm_moveit_planner_id_combo.setVisible(
                arm_backend_is_moveit and arm_pipeline_is_pilz
            )

        full_backend_is_moveit = self._get_planner_backend(context='full') == 'moveit'
        full_pipeline_is_pilz = (
            hasattr(self, 'full_control_moveit_pipeline_combo')
            and self.full_control_moveit_pipeline_combo.currentText()
            == 'pilz_industrial_motion_planner'
        )
        if hasattr(self, 'full_control_moveit_pipeline_label'):
            self.full_control_moveit_pipeline_label.setVisible(full_backend_is_moveit)
        if hasattr(self, 'full_control_moveit_pipeline_combo'):
            self.full_control_moveit_pipeline_combo.setVisible(full_backend_is_moveit)
        if hasattr(self, 'full_control_moveit_planner_id_label'):
            self.full_control_moveit_planner_id_label.setVisible(
                full_backend_is_moveit and full_pipeline_is_pilz
            )
        if hasattr(self, 'full_control_moveit_planner_id_combo'):
            self.full_control_moveit_planner_id_combo.setVisible(
                full_backend_is_moveit and full_pipeline_is_pilz
            )

    def _get_active_planner_context(self):
        """Use the currently open planner tab, falling back to the last Arm/Full tab visited."""
        if hasattr(self, 'tabs'):
            current_index = self.tabs.currentIndex()
            if hasattr(self, 'ARM_TAB_INDEX') and current_index == self.ARM_TAB_INDEX:
                self._active_planner_context = 'arm'
            elif hasattr(self, 'FULL_CONTROL_TAB_INDEX') and current_index == self.FULL_CONTROL_TAB_INDEX:
                self._active_planner_context = 'full'

        return getattr(self, '_active_planner_context', 'arm')

    def _update_joint_control_tab_state(self):
        """Disable Joint Control only when moveit is selected in the active planner tab."""
        planner_context = self._get_active_planner_context()
        planner_backend = self._get_planner_backend(context=planner_context)
        is_moveit_active = (planner_backend == 'moveit')

        if hasattr(self, 'tabs') and hasattr(self, 'JOINT_TAB_INDEX'):
            self.tabs.setTabEnabled(self.JOINT_TAB_INDEX, not is_moveit_active)

    def _update_full_control_init_box_state(self):
        """Disable Full Control initialization only when sim=true and hybrid_sim=false."""
        full_sim_mode = self.full_control_sim_mode_combo.currentText() if hasattr(self, "full_control_sim_mode_combo") else 'false'
        full_hybrid_mode = self._sync_full_control_hybrid_sim_state()
        should_disable = (full_sim_mode == 'true' and full_hybrid_mode == 'false')
        if hasattr(self, "full_control_init_box"):
            self.full_control_init_box.setEnabled(not should_disable)

    def _update_headless_visibility(self):
        """Show simulation-only selectors only when simulation mode is true."""
        arm_sim_mode = self.arm_sim_mode_combo.currentText() if hasattr(self, "arm_sim_mode_combo") else 'false'
        arm_planner = self._get_planner_backend(context='arm')
        # URSim selector is always visible when planner_backend is moveit (must be locked true)
        arm_ursim_visible = (arm_sim_mode == 'true') or (arm_planner == 'moveit')
        if hasattr(self, "arm_hybrid_sim_combo") and not arm_ursim_visible:
            self.arm_hybrid_sim_combo.setCurrentText('false')
        if hasattr(self, "arm_hybrid_sim_label"):
            self.arm_hybrid_sim_label.setVisible(arm_ursim_visible)
        if hasattr(self, "arm_hybrid_sim_combo"):
            self.arm_hybrid_sim_combo.setVisible(arm_ursim_visible)

        base_sim_mode = self.sim_mode_combo.currentText() if hasattr(self, "sim_mode_combo") else 'false'
        base_visible = (base_sim_mode == 'true')
        if hasattr(self, "base_headless_label"):
            self.base_headless_label.setVisible(base_visible)
        if hasattr(self, "base_headless_combo"):
            self.base_headless_combo.setVisible(base_visible)

        full_sim_mode = self.full_control_sim_mode_combo.currentText() if hasattr(self, "full_control_sim_mode_combo") else 'false'
        full_visible = (full_sim_mode == 'true')
        full_hybrid_mode = self._sync_full_control_hybrid_sim_state()
        if hasattr(self, "full_control_headless_label"):
            self.full_control_headless_label.setVisible(full_visible)
        if hasattr(self, "full_control_headless_combo"):
            self.full_control_headless_combo.setVisible(full_visible)

        # Full Control hybrid selector is only meaningful in sim mode.
        if hasattr(self, "full_control_hybrid_sim_label"):
            self.full_control_hybrid_sim_label.setVisible(full_visible)
        if hasattr(self, "full_control_hybrid_sim_combo"):
            self.full_control_hybrid_sim_combo.setVisible(full_visible)

        # In Full Control, headless is only meaningful when hybrid_sim is false.
        full_headless_visible = full_visible and (full_hybrid_mode == 'false')
        if hasattr(self, "full_control_headless_label"):
            self.full_control_headless_label.setVisible(full_headless_visible)
        if hasattr(self, "full_control_headless_combo"):
            self.full_control_headless_combo.setVisible(full_headless_visible)

    def _set_tab_enabled(self, tab_index, enabled):
        """Enable or disable a tab (make it clickable or unclickable)"""
        self.tabs.setTabEnabled(tab_index, enabled)

    def _update_tab_states_for_base(self):
        """Update tab states based on base control processes"""
        # Any base (non-full) process running?
        base_running = any(self._is_base_tab_state_process(key) for key in self.process_map)

        if base_running:
            # Disable Arm and Full Control tabs (Base stays enabled, Joint stays enabled)
            self._set_tab_enabled(self.ARM_TAB_INDEX, False)
            self._set_tab_enabled(self.FULL_CONTROL_TAB_INDEX, False)
        else:
            # Re‑enable when base processes stop
            self._set_tab_enabled(self.ARM_TAB_INDEX, True)
            self._set_tab_enabled(self.FULL_CONTROL_TAB_INDEX, True)

    def _update_tab_states_for_full_control(self):
        """Update tab states based on full control processes"""
        full_running = any(
            key in self.process_map
            for key in [
                'full_mobile_manipulator',
                'full_mapping',
                'full_localization',
                'full_nav2',
                'full_exploration',
            ]
        )

        if full_running:
            # Disable Base, Arm, and Joint tabs (Full Control stays enabled)
            self._set_tab_enabled(self.BASE_TAB_INDEX, False)
            self._set_tab_enabled(self.ARM_TAB_INDEX, False)
            # self._set_tab_enabled(self.JOINT_TAB_INDEX, False)
        else:
            # Re‑enable them when full control processes stop
            self._set_tab_enabled(self.BASE_TAB_INDEX, True)
            self._set_tab_enabled(self.ARM_TAB_INDEX, True)
            # self._set_tab_enabled(self.JOINT_TAB_INDEX, True)


    def _update_init_box_state(self):
        """Disable Arm initialization only when sim=true and URsim=false."""
        sim_mode = self.arm_sim_mode_combo.currentText()
        hybrid_mode = self.arm_hybrid_sim_combo.currentText() if hasattr(self, "arm_hybrid_sim_combo") else 'false'
        should_disable = (sim_mode == 'true' and hybrid_mode == 'false')
        self.init_box.setEnabled(not should_disable)

    def _update_tab_states_for_arm(self):
        """Update tab states based on arm control processes"""
        arm_running = 'arm_launch' in self.process_map

        if arm_running:
            # Disable Base and Full Control tabs only
            self._set_tab_enabled(self.BASE_TAB_INDEX, False)
            self._set_tab_enabled(self.FULL_CONTROL_TAB_INDEX, False)
            # Joint tab remains enabled
        else:
            self._set_tab_enabled(self.BASE_TAB_INDEX, True)
            self._set_tab_enabled(self.FULL_CONTROL_TAB_INDEX, True)

    def _spin_ros(self):
        """Safely spin ROS, checking context is valid first"""
        try:
            if rclpy.ok():
                rclpy.spin_once(self.node, timeout_sec=0)
        except Exception:
            pass  # Ignore errors if context is shutting down

    def _on_joint_states(self, msg, source_topic=None):
        """Update live joint sliders in Arm and Full Control tabs from joint states topics."""
        if source_topic is not None:
            arm_topic = self._get_joint_states_topic_for_ui(context='arm')
            full_topic = self._get_joint_states_topic_for_ui(context='full')
            if source_topic not in (arm_topic, full_topic):
                return

        if not msg.name or not msg.position:
            return

        joint_position_map = dict(zip(msg.name, msg.position))
        updated_positions = {}

        monitor_sets = [
            ('arm_control_joint_sliders', 'arm_control_joint_value_labels'),
            ('full_control_joint_sliders', 'full_control_joint_value_labels'),
        ]

        for slider_attr, label_attr in monitor_sets:
            if not hasattr(self, slider_attr) or not hasattr(self, label_attr):
                continue

            sliders = getattr(self, slider_attr)
            labels = getattr(self, label_attr)

            for i, joint_name in enumerate(self.arm_joint_names):
                if joint_name not in joint_position_map:
                    continue
                if i >= len(sliders) or i >= len(labels):
                    continue

                position = joint_position_map[joint_name]
                slider = sliders[i]
                slider_value = int(position * 100)
                slider_value = max(slider.minimum(), min(slider.maximum(), slider_value))
                slider.setValue(slider_value)
                labels[i].setText(f"{position:.3f} rad")
                updated_positions[joint_name] = position

        if len(updated_positions) == len(self.arm_joint_names):
            self.current_joint_positions = updated_positions
    
    def _on_tab_changed(self, index, tabs):
        """Handle tab change - check joint states when Joint Control tab is activated"""
        self._update_joint_control_tab_state()

        # Check if the Joint Control tab (index 2) is now active
        if index == 2 and tabs.tabText(index) == "Joint Control":
            # Disable controls first for safety
            for input_field in self.joint_inputs:
                input_field.setEnabled(False)
            for slider in self.joint_sliders:
                slider.setEnabled(False)
            self.btn_publish_joints.setEnabled(False)
            
            # Automatically read current joint positions to verify topic is still publishing
            self.joint_status_text.insertHtml("<b style='color: #539bf5;'> Checking for current joint positions...</b>")
            self.read_joint_positions()
 
    @staticmethod
    def _strip_html(text):
        """Strip HTML tags to get plain text for filter matching."""
        import re as _re
        return _re.sub(r'<[^>]+>', '', text)

    def _log_append(self, widget, content, add_newline=True):
        """Store entry in the per-widget log buffer and render it if it passes the current filter."""
        plain = self._strip_html(content)
        self.tab_log_entries.setdefault(id(widget), []).append((content, plain, add_newline))
        filt = self.tab_filter_inputs.get(id(widget))
        term = filt.text().strip().lower() if filt else ''
        if not term or term in plain.lower():
            self._append_to_text_widget(widget, content, add_newline)

    def _log_apply_filter(self, widget):
        """Rebuild the widget showing only entries whose plain text matches the current filter."""
        filt = self.tab_filter_inputs.get(id(widget))
        term = filt.text().strip().lower() if filt else ''
        scrollbar = widget.verticalScrollBar()
        was_at_bottom = scrollbar.value() >= scrollbar.maximum() - 10
        widget.clear()
        for h, plain, nl in self.tab_log_entries.get(id(widget), []):
            if not term or term in plain.lower():
                self._append_to_text_widget(widget, h, nl)
        if was_at_bottom:
            scrollbar.setValue(scrollbar.maximum())

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
    
    def _append_to_text_widget(self, text_widget, html_content, add_newline=True):
        """
        Append content to a text widget with smart scrolling.
        Only auto-scrolls if the user is already viewing the bottom.
        
        Args:
            text_widget: QTextEdit widget to append to
            html_content: HTML content to append
            add_newline: Whether to add a newline after the content
        """
        # Check if scrollbar is at the bottom before appending
        scrollbar = text_widget.verticalScrollBar()
        was_at_bottom = scrollbar.value() >= scrollbar.maximum() - 10  # Small tolerance for rounding
        
        # Save the current scroll position
        old_scroll_value = scrollbar.value()
        
        # Append the content
        cursor = text_widget.textCursor()
        cursor.movePosition(cursor.End)
        cursor.insertHtml(html_content)
        if add_newline:
            cursor.insertText('\n')
        
        # Restore scroll position or scroll to bottom as appropriate
        if was_at_bottom:
            # User was at bottom, scroll to new bottom
            scrollbar.setValue(scrollbar.maximum())
        else:
            # User was scrolled up, maintain their position
            scrollbar.setValue(old_scroll_value)

    def _build_gpr_request_groups(self):
        """Return the supported GPR API requests grouped for the UI."""
        probe_status_tooltip = """Get information about Probe. The response **must** include:

- If GP App is connected to a Probe. If that’s the case:
    
    - Serial Number of the Probe
        
    - Probe model (GP8000, GP8100, GP8800)
        
    - Battery status of the Probe
        

If no probe is connected, the probe field should be absent""".strip()

        probe_connect_tooltip = """Connect to Probe at a given IP address (optional) and serial number.

This endpoint should wait until connection is successful or fails before
returning. The app should not show any alert in case of failure.

If no contract is found, or if contract does not have API access feature
flag, the app should disconnect from the probe immediately and return a
403 error.

If an IP was specified, and connection was successful, but serial number
does not match the one that was provided, app should disconnect and
throw an error.""".strip()

        line_get_tooltip = """Gets the current line being measured.

If the measurement has not been started, return index: 0 and started:
false.""".strip()

        presets_tooltip = """Set measurement parameters:

- Resolution (max depth / max speed)
    
- Repetition rate (number, time triggered, or “rising edge” (GP8800
only))
    

This endpoint can only be called before any data has been added to a
measurement.

#### Fixed step size

``` json
{
    "mode": "FIXED_STEP",
    "stepSize": 0.5, // For metric, value in [scans/cm]. For imperial value in [scans/in],
    "unit": "METRIC" | "IMPERIAL",
    "resolution": "MAX_SPEED" | "MAX_DEPTH" // only available with GP8000
}
```

#### Time-triggered

``` json
{
    "mode": "TIMED",
    "scanFrequency": 0.5 // Value in scans / second
    "unit": "METRIC" | "IMPERIAL",
    "resolution": "MAX_SPEED" | "MAX_DEPTH" // only available with GP8000
}
```

#### External trigger (GP8800 only)

``` json
{
    "mode": "EXTERNAL"
}
```""".strip()

        measurement_start_tooltip = """Start a new measurement. The endpoint **must** allow to create one of
those measurement types:

- Line scan
    
- Superline scan (GP8100 only)
    
- Area scan
    

The endpoint will return when the the measurement is ready to start and
a line can be started. The app will navigate to the measurements screen.

If a new measurement has already been started, but no data has been
added to it, it should be discarded. If data has been added, an error
should be thrown (see below) until the measurement has been stopped.
This is similar to the current behavior in the app when pressing the “+”
button or going back to explorer screen.""".strip()

        display_tooltip = """Set display parameters:

- View to apply to
    
- Color map
    
    - Scheme name
        
    - Brightness
        
- Slice start and end (C-Scan and superline view only)
    

Setting the parameters for a given view must switch the app to that view
and apply the settings. The applied display parameters are then to be
used for image generation.""".strip()

        processing_tooltip = """Set image processing parameters:

- View to apply to
    
- Gain
    
    - Auto gain (on/off)
        
    - Adjust gain (press “ok” button)
        
    - Linear gain (dB)
        
    - Time gain compensation (dB/ns)
        
- Noise cancellation (on/off)
    
- Background removal depth (ns)
    
- Depth / time window (ns)
    
- Dielectric constant
    

This endpoint can only be called at any time. If the gain object is
present, the view must change to reflect that which is specified in the
view property.""".strip()

        export_bscan_tooltip = """Get a B-Scan as a PNG image.

Grid lines, marker lines and tags, must not be visible. This should be a
raw image of the migrated B-Scan data.

- For a superline scan (GP8100 only), given a channel (A-F).
    
- For an area scan, given a line / column ID
    
    - For GP8100, an additional channel (A-F) is required
        

The endpoint **must** use the post-processing and display parameters for
the MIGRATED_B_SCAN view:

- Color map (scheme + brightness)
    
- Gain
    
- Noise cancellation
    
- Background removal
    
- Depth / time window (GP8000 only)
    

The image returned will be of the native resolution for the device,
meaning:

- **Width:** amount of a-scans for the line
    
- **Height:** amount of samples for an a-scan
    

This endpoint can be called at any time if data is available for the
specified channel / line combination.""".strip()

        export_cscan_tooltip = """Get a C-Scan as a PNG image.

This is only available in Area Scan.

Grid lines, marker lines and tags, must not be visible. This should be a
raw image of C-Scan

The endpoint **must** use the post-processing and display parameters for
the C_SCAN view:

- Color map (scheme + brightness)
    
- Gain
    
- Noise cancellation
    
- Background removal
    
- Depth / time window (GP8000 only)
    

The image returned will be of the resolution provided by the user

This endpoint can be called at any time if data is available.""".strip()

        export_ascan_tooltip = """Get an A-Scan as a CSV.

- For a line scan, given a distance from the start.
    
- For a superline scan (GP8100 only), given a distance from the start
and a channel (A-F)
    
- For an area scan, given a line / column ID and a distance from the
start of the line / column (= X and Y coordinate)
    
    - For GP8100, an additional channel (A-F) is required
        
- The returned CSV contains the full raw data of a-scan.""".strip()

        return {
            'probe': [
                {
                    'label': 'Connect to probe',
                    'method': 'POST',
                    'path': '/probe/connect',
                    'body': {'serialNumber': 'GP88-007-0081'},
                    'description': 'Connect to the GPR probe using the provided serial number.',
                    'tooltip': probe_connect_tooltip,
                },
                {
                    'label': 'Disconnect probe',
                    'method': 'POST',
                    'path': '/probe/disconnect',
                    'body': None,
                    'description': 'Disconnect from the currently connected probe.',
                    'tooltip': "Disconnects from the currently connected probe.",
                },
                {
                    'label': 'Get probe status',
                    'method': 'GET',
                    'path': '/probe',
                    'body': None,
                    'description': 'Get the current probe connection status and probe details when connected.',
                    'tooltip': probe_status_tooltip,
                },
            ],
            'line': [
                {
                    'label': 'Start line',
                    'method': 'POST',
                    'path': '/measurement/line/start',
                    'body': None,
                    'description': 'Start the current line. The probe should wake up if needed.',
                    'tooltip': """Start line.

This works for any measurement type. If the device is in sleep mode, it
should wake it up and return only when the device is ready and a-scans
can be fed to the app.""".strip(),
                },
                {
                    'label': 'Stop line',
                    'method': 'POST',
                    'path': '/measurement/line/stop',
                    'body': None,
                    'description': 'Stop the current line. In Area Scan it does not advance until the next line is started.',
                    'tooltip': """Stop line. In area scan, this should **not** move to the next line until
it has been started.

This works for any measurement type. If the device is in sleep mode, it
should wake it up and return only when the device is ready and a-scans
can be fed to the app.""".strip(),
                },
                {
                    'label': 'Pause line',
                    'method': 'POST',
                    'path': '/measurement/line/pause',
                    'body': None,
                    'description': 'Pause the current measurement line.',
                    'tooltip': """Pauses current line.

This works for any measurement type. If the device is in sleep mode, it
should wake it up and return only when the device is ready and a-scans
can be fed to the app.""".strip(),
                },
                {
                    'label': 'Unpause line',
                    'method': 'POST',
                    'path': '/measurement/line/unpause',
                    'body': None,
                    'description': 'Resume a paused measurement line.',
                    'tooltip': """Unpauses current line.

Only works if the current line has been paused""".strip(),
                },
                {
                    'label': 'Get current line',
                    'method': 'GET',
                    'path': '/measurement/line',
                    'body': None,
                    'description': 'Get the current line status, scan count, length, and line index.',
                    'tooltip': line_get_tooltip,
                },
                {
                    'label': 'Delete current line',
                    'method': 'DELETE',
                    'path': '/measurement/line',
                    'body': None,
                    'description': 'Delete the current line and move back to the previous line. Area Scan only.',
                    'tooltip': """Deletes the current line being measured. Sets the current line to the
previous line.

Only available for Area Scan.""".strip(),
                },
            ],
            'presets': [
                {
                    'label': 'Set Fixed Step',
                    'method': 'POST',
                    'path': '/measurement/presets',
                    'body': {'mode': 'FIXED_STEP', 'repetitionRate': 0.5, 'unit': 'METRIC'},
                    'description': 'Set measurement presets for fixed-step acquisition before data is added.',
                    'tooltip': presets_tooltip,
                },
                {
                    'label': 'Set Time Trigger',
                    'method': 'POST',
                    'path': '/measurement/presets',
                    'body': {'mode': 'TIMED', 'scanFrequency': 0.5, 'unit': 'METRIC'},
                    'description': 'Set measurement presets for time-triggered acquisition. Swagger examples use scanFrequency for this mode.',
                    'tooltip': presets_tooltip,
                },
                {
                    'label': 'Set External Trigger',
                    'method': 'POST',
                    'path': '/measurement/presets',
                    'body': {'mode': 'EXTERNAL'},
                    'description': 'Set measurement presets for external triggering. Available for GP8800 according to the Swagger examples.',
                    'tooltip': presets_tooltip,
                },
            ],
            'measurements': [
                {
                    'label': 'Start measurement',
                    'method': 'POST',
                    'path': '/measurement/start',
                    'body': {'type': 'LINE_SCAN', 'name': 'GPR API Test'},
                    'description': 'Create and start a new measurement session.',
                    'tooltip': measurement_start_tooltip,
                },
                {
                    'label': 'Stop measurement',
                    'method': 'POST',
                    'path': '/measurement/stop',
                    'body': None,
                    'description': 'Stop the current measurement and leave the measurement screen.',
                    'tooltip': """Stop the measurement. This will exit from the measurement screen and
return to the home screen.""".strip(),
                },
                {
                    'label': 'Delete measurement',
                    'method': 'DELETE',
                    'path': '/measurement',
                    'body': None,
                    'description': 'Delete the current measurement. The API description says a new measurement is created afterward.',
                    'tooltip': "Deletes the current measurement and creates a new one.",
                },
                {
                    'label': 'Trigger sync',
                    'method': 'POST',
                    'path': '/measurement/sync',
                    'body': None,
                    'description': 'Sync the current measurement to Workspace after the line has been stopped.',
                    'tooltip': """Triggers a sync of the current measurement to Workspace.

This endpoint can only be called when line has been stopped.

This endpoint locks the UI of the GP App during syncing. Requires GP App
to have access to the internet.""".strip(),
                },
                {
                    'label': 'Get measurement info',
                    'method': 'GET',
                    'path': '/measurement/info',
                    'body': None,
                    'description': 'Get current measurement metadata and settings.',
                    'tooltip': "Gets the current measurement information",
                },
                {
                    'label': 'Set display params',
                    'method': 'POST',
                    'path': '/measurement/display',
                    'body': {'view': 'B_SCAN', 'colorMap': {'scheme': 'COOL', 'brightness': 0}},
                    'description': 'Set display parameters for a measurement view, such as the active colormap.',
                    'tooltip': display_tooltip,
                },
                {
                    'label': 'Set processing params',
                    'method': 'POST',
                    'path': '/measurement/processing',
                    'body': {
                        'gain': {'view': 'B_SCAN', 'autoGain': True},
                        'noiseCancellation': True,
                        'backgroundRemovalDepth': 0,
                        'dielectricConstant': 6,
                    },
                    'description': 'Set image processing parameters such as gain, noise cancellation, and dielectric constant.',
                    'tooltip': processing_tooltip,
                },
            ],
            'export': [
                {
                    'label': 'Export raw data',
                    'method': 'POST',
                    'path': '/measurement/export/raw',
                    'body': None,
                    'description': 'Export raw measurement data as an offline SEG-Y zip package.',
                    'download_extension': 'zip',
                    'tooltip': """Exports raw data of the measurement.

Generates a local (offline) SEG-Y export of the entire measurement. The
result is a zip file containing all b-scans, along with a CSV.""".strip(),
                },
                {
                    'label': 'Get B-Scan',
                    'method': 'POST',
                    'path': '/measurement/export/bscan',
                    'body': {'migrated': True},
                    'description': 'Export a B-Scan PNG. Additional channel or line fields may be needed for some measurement types.',
                    'download_extension': 'png',
                    'tooltip': export_bscan_tooltip,
                },
                {
                    'label': 'Get C-Scan',
                    'method': 'POST',
                    'path': '/measurement/export/cscan',
                    'body': {'width': 300, 'height': 300},
                    'description': 'Export a C-Scan PNG. This is only available for Area Scan measurements.',
                    'download_extension': 'png',
                    'tooltip': export_cscan_tooltip,
                },
                {
                    'label': 'Get A-Scan',
                    'method': 'POST',
                    'path': '/measurement/export/ascan',
                    'body': {'index': 0},
                    'description': 'Export an A-Scan CSV. Additional channel or line fields may be needed for some measurement types.',
                    'download_extension': 'csv',
                    'tooltip': export_ascan_tooltip,
                },
            ],
        }

    def _create_gpr_api_test_tab(self):
        """Create the GPR API test tab UI."""
        gpr_tab = QWidget()
        gpr_tab_layout = QVBoxLayout(gpr_tab)

        gpr_base_url_layout = QHBoxLayout()
        gpr_base_url_layout.addWidget(QLabel("Base URL:"))
        self.gpr_base_url_input = QLineEdit("http://192.168.42.53:9000")
        self.gpr_base_url_input.setPlaceholderText("http://192.168.42.53:9000")
        self.gpr_base_url_input.setToolTip("Base URL for the GPR HTTP server.")
        gpr_base_url_layout.addWidget(self.gpr_base_url_input)
        gpr_base_url_layout.addStretch()
        gpr_tab_layout.addLayout(gpr_base_url_layout)

        gpr_groups_layout = QGridLayout()
        gpr_groups_layout.setHorizontalSpacing(12)
        gpr_groups_layout.setVerticalSpacing(12)
        group_positions = [
            ('probe', 'Probe', 0, 0),
            ('line', 'Line', 0, 1),
            ('presets', 'Presets', 0, 2),
            ('measurements', 'Measurements', 1, 0),
            ('export', 'Export', 1, 1),
        ]
        for group_key, title, row, col in group_positions:
            gpr_groups_layout.addWidget(self._create_gpr_group_box(group_key, title), row, col)
        gpr_groups_layout.setColumnStretch(0, 1)
        gpr_groups_layout.setColumnStretch(1, 1)
        gpr_groups_layout.setColumnStretch(2, 1)
        gpr_tab_layout.addLayout(gpr_groups_layout)

        self.gpr_status_text = QTextEdit()
        self.gpr_status_text.setReadOnly(True)
        self.gpr_status_text.setAcceptRichText(True)
        self.gpr_status_text.setStyleSheet(
            "background-color: #22272e; color: #adbac7; border: 1px solid #444c56; "
            "font-family: 'Courier New', monospace; white-space: pre;"
        )
        font_metrics = QFontMetrics(self.gpr_status_text.font())
        tab_width = font_metrics.horizontalAdvance(' ') * 8
        self.gpr_status_text.setTabStopDistance(tab_width)

        gpr_status_header = QHBoxLayout()
        gpr_status_header.addWidget(QLabel("GPR API Test - Curl Command + Response"))
        gpr_status_header.addStretch()
        gpr_status_header.addWidget(QLabel("Filter:"))
        self.gpr_search_input = QLineEdit()
        self.gpr_search_input.setPlaceholderText("Filter output lines...")
        self.gpr_search_input.setMaximumWidth(200)
        self.gpr_search_input.textChanged.connect(lambda: self._log_apply_filter(self.gpr_status_text))
        gpr_status_header.addWidget(self.gpr_search_input)
        self.tab_filter_inputs[id(self.gpr_status_text)] = self.gpr_search_input

        self.btn_restore_gpr_status = QPushButton("Restore")
        self.btn_restore_gpr_status.clicked.connect(self.restore_gpr_status)
        self.btn_restore_gpr_status.setMaximumWidth(80)
        self.btn_restore_gpr_status.setVisible(False)
        gpr_status_header.addWidget(self.btn_restore_gpr_status)

        btn_clear_gpr_status = QPushButton("Clear")
        btn_clear_gpr_status.clicked.connect(self.clear_gpr_status)
        btn_clear_gpr_status.setMaximumWidth(80)
        gpr_status_header.addWidget(btn_clear_gpr_status)

        gpr_tab_layout.addLayout(gpr_status_header)
        gpr_tab_layout.addWidget(self.gpr_status_text, 1)
        return gpr_tab

    def _create_gpr_group_box(self, group_key, title):
        """Create a labeled group box with a request combobox and send button."""
        group_box = QGroupBox(title)
        group_layout = QVBoxLayout()
        group_box.setLayout(group_layout)

        request_combo = QComboBox()
        request_combo.setMinimumWidth(220)
        self._populate_gpr_request_combo(request_combo, self.gpr_request_groups[group_key])
        group_layout.addWidget(request_combo)

        send_button = QPushButton("Send")
        send_button.clicked.connect(
            lambda _, combo=request_combo, button=send_button: self.send_gpr_request(combo, button)
        )
        group_layout.addWidget(send_button)
        group_layout.addStretch()

        self.gpr_group_combos[group_key] = request_combo
        return group_box

    def _populate_gpr_request_combo(self, combo, requests):
        """Populate a GPR request combobox with tooltip metadata."""
        for request in requests:
            combo.addItem(request['label'], request)
            item_index = combo.count() - 1
            combo.setItemData(item_index, request.get('tooltip', request['description']), Qt.ToolTipRole)

        combo.currentIndexChanged.connect(lambda _, c=combo: self._update_gpr_combo_tooltip(c))
        self._update_gpr_combo_tooltip(combo)

    def _update_gpr_combo_tooltip(self, combo):
        """Keep the combobox tooltip aligned with the selected request."""
        request = combo.currentData()
        combo.setToolTip(request.get('tooltip', request['description']) if request else "")

    def _get_gpr_download_dir(self):
        """Return the directory used for downloaded GPR API artifacts."""
        return os.path.expanduser('~/Downloads/GP_API_Test')

    def _build_gpr_download_path(self, request):
        """Build a timestamped download path for file-based GPR API responses."""
        download_dir = self._get_gpr_download_dir()
        safe_stem = request['path'].strip('/').replace('/', '_') or 'gpr_download'
        timestamp = time.strftime('%Y%m%d_%H%M%S')
        extension = request['download_extension']
        return os.path.join(download_dir, f'{safe_stem}_{timestamp}.{extension}')

    def _extract_gpr_http_status(self, output_text):
        """Extract the curl write-out HTTP status code from buffered output."""
        match = re.search(r'HTTP_STATUS:(\d{3})', output_text or '')
        return int(match.group(1)) if match else None

    def send_gpr_request(self, combo, button):
        """Send the selected GPR HTTP request using curl."""
        request = combo.currentData()
        if not request:
            self._log_append(
                self.gpr_status_text,
                "<span style='color: #c69026;'>⚠ No GPR request selected</span>",
            )
            return

        base_url = self.gpr_base_url_input.text().strip().rstrip('/')
        if not base_url:
            self._log_append(
                self.gpr_status_text,
                "<span style='color: #c69026;'>⚠ Base URL is empty</span>",
            )
            return

        url = f"{base_url}{request['path']}"
        output_path = None
        if request.get('download_extension'):
            download_dir = self._get_gpr_download_dir()
            try:
                os.makedirs(download_dir, exist_ok=True)
            except OSError as exc:
                self._log_append(
                    self.gpr_status_text,
                    f"<span style='color: #f47067;'>✗ Could not create download directory {html.escape(download_dir)}: {html.escape(str(exc))}</span>",
                )
                return
            output_path = self._build_gpr_download_path(request)

        args = [
            '--silent',
            '--show-error',
            '--location',
            '--max-time',
            '30',
            '--request',
            request['method'],
            url,
            '--write-out',
            '\nHTTP_STATUS:%{http_code}\n',
        ]

        if request['body'] is not None:
            payload = json.dumps(request['body'], separators=(',', ':'))
            args.extend([
                '--header',
                'Content-Type: application/json',
                '--data',
                payload,
            ])

        if output_path:
            args.extend(['--output', output_path])

        cmd_str = 'curl ' + ' '.join(shlex.quote(arg) for arg in args)
        self._log_append(
            self.gpr_status_text,
            f"<b style='color: #57ab5a;'>▶ {html.escape(cmd_str)}</b>",
        )
        self._log_append(
            self.gpr_status_text,
            f"<span style='color: #76e3ea;'>Request: {html.escape(request['description'])}</span>",
        )
        if output_path:
            self._log_append(
                self.gpr_status_text,
                f"<span style='color: #76e3ea;'>Saving response to: {html.escape(output_path)}</span>",
            )

        process = QProcess(self)
        process.setProcessChannelMode(QProcess.MergedChannels)
        process.setProperty('had_output', False)
        process.setProperty('gpr_output_buffer', '')
        process.setProperty('gpr_output_path', output_path or '')
        process.readyReadStandardOutput.connect(lambda p=process: self.handle_gpr_output(p))
        process.finished.connect(
            lambda exit_code, exit_status, p=process, b=button: self._on_gpr_request_finished(
                p, exit_code, exit_status, b
            )
        )

        button.setEnabled(False)
        button.setText("Sending...")
        process.start('curl', args)
        self.gpr_request_processes[id(process)] = process

    def handle_gpr_output(self, process):
        """Append curl output for GPR requests to the GPR status pane."""
        output = process.readAllStandardOutput().data().decode(errors='replace')
        if not output:
            return

        process.setProperty('had_output', True)
        previous_output = process.property('gpr_output_buffer') or ''
        process.setProperty('gpr_output_buffer', previous_output + output)
        self._log_append(
            self.gpr_status_text,
            (
                "<pre style=\"margin: 0; font-family: 'Courier New', monospace;\">"
                f"{html.escape(output)}</pre>"
            ),
            add_newline=False,
        )

    def _on_gpr_request_finished(self, process, exit_code, exit_status, button):
        """Restore the button state and log the curl completion status."""
        self.gpr_request_processes.pop(id(process), None)
        button.setEnabled(True)
        button.setText("Send")
        output_path = process.property('gpr_output_path') or ''
        output_text = process.property('gpr_output_buffer') or ''
        http_status = self._extract_gpr_http_status(output_text)

        if exit_code == 0:
            if http_status is not None and http_status >= 400:
                if output_path and os.path.exists(output_path):
                    try:
                        os.remove(output_path)
                    except OSError:
                        pass
                self._log_append(
                    self.gpr_status_text,
                    f"<span style='color: #f47067;'>✗ Request completed with HTTP status {http_status}</span>",
                )
            elif output_path:
                self._log_append(
                    self.gpr_status_text,
                    f"<span style='color: #57ab5a;'>✓ Saved response to {html.escape(output_path)}</span>",
                )
            elif not bool(process.property('had_output')):
                self._log_append(
                    self.gpr_status_text,
                    "<span style='color: #57ab5a;'>✓ Request completed (no response body)</span>",
                )
            else:
                self._log_append(
                    self.gpr_status_text,
                    "<span style='color: #57ab5a;'>✓ Request completed</span>",
                )
        else:
            if output_path and os.path.exists(output_path):
                try:
                    os.remove(output_path)
                except OSError:
                    pass
            self._log_append(
                self.gpr_status_text,
                (
                    "<span style='color: #f47067;'>"
                    f"✗ curl exited with code {exit_code}</span>"
                ),
            )

        self._log_append(self.gpr_status_text, "")
        process.deleteLater()

    def clear_gpr_status(self):
        widget = self.gpr_status_text
        self.tab_log_backups[id(widget)] = list(self.tab_log_entries.get(id(widget), []))
        self.tab_log_entries[id(widget)] = []
        widget.clear()
        self.btn_restore_gpr_status.setVisible(True)

    def restore_gpr_status(self):
        widget = self.gpr_status_text
        backup = self.tab_log_backups.get(id(widget))
        if backup:
            current = self.tab_log_entries.get(id(widget), [])
            self.tab_log_entries[id(widget)] = backup + current
            self.tab_log_backups[id(widget)] = []
            self._log_apply_filter(widget)
            self.btn_restore_gpr_status.setVisible(False)

    def toggle_arm_launch(self):
        sim_mode = self.arm_sim_mode_combo.currentText()
        hybrid_sim = 'true' if self._is_hybrid_sim_enabled(context='arm') else 'false'
        robot_ip = self._get_robot_ip_for_launch(context='arm')
        planner_backend = self._get_planner_backend(context='arm')
        namespace_arm = self._get_namespace_for_arm_launch(planner_backend)
        launch_args = [
            'launch', 'arm_control', 'arm.launch.py',
            f'robot_ip:={robot_ip}',
            f'sim:={sim_mode}',
            f'hybrid_sim:={hybrid_sim}',
            f'planner_backend:={planner_backend}',
            'mode:=arm',
        ]
        launch_args.extend(self._get_moveit_launch_args(context='arm'))
        if namespace_arm:
            launch_args.append(f'namespace_arm:={namespace_arm}')
        self._toggle_process('arm_launch', self.btn_general_launch, 'Arm',
                            'ros2', launch_args)
        
        # Update tab states
        self._update_tab_states_for_arm()

    def toggle_base_control_launch(self):
        """Toggle the base-only bringup from the Base Control tab."""
        sim_mode = self.sim_mode_combo.currentText()
        controller_type = self.controller_type_combo.currentText()
        headless = self.base_headless_combo.currentText()

        launch_args = [
            'launch', 'navi_wall', 'platform.launch.py',
            f'sim:={sim_mode}',
            'mode:=base',
            f'controller_type:={controller_type}',
            'odom_tf_from_controller:=true',
            'publish_controller_odom_tf:=true',
            'launch_rviz:=true',
            f'headless:={headless}',
        ]
        if 'mobile_platform' not in self.process_map:
            self.btn_launch_base_robot.setProperty('uses_gazebo', sim_mode == 'true')

        self._toggle_base_process(
            'mobile_platform',
            self.btn_launch_base_robot,
            'Base Robot',
            'ros2',
            launch_args,
        )
        self._update_tab_states_for_base()

    def toggle_full_control_launch(self):
        """Toggle the full mobile manipulator bringup from the Full Control tab."""
        sim_mode = self.full_control_sim_mode_combo.currentText()
        hybrid_sim = self._sync_full_control_hybrid_sim_state()
        launch_file = self._get_full_control_launch_file(hybrid_sim=hybrid_sim)
        controller_type = self.full_control_controller_type_combo.currentText()
        publish_controller_odom_tf = 'true'
        headless = self.full_control_headless_combo.currentText()
        robot_ip = self._get_robot_ip_for_launch(context='full')

        launch_args = [
            'launch', 'navi_wall', launch_file,
            'mode:=full',
            f'robot_ip:={robot_ip}',
            f'controller_type:={controller_type}',
            f'publish_controller_odom_tf:={publish_controller_odom_tf}',
            f'planner_backend:={self._get_planner_backend(context="full")}',
            f'headless:={headless}',
        ]
        if launch_file == 'oliwall_mobile_manipulator.launch.py':
            launch_args.extend([
                f'sim:={sim_mode}',
                f'hybrid_sim:={hybrid_sim}',
            ])
        launch_args.extend(self._get_moveit_launch_args(context='full'))
        if 'full_mobile_manipulator' not in self.process_map:
            self.btn_full_control_launch.setProperty('uses_gazebo', sim_mode == 'true')

        self._toggle_base_process(
            'full_mobile_manipulator',
            self.btn_full_control_launch,
            'Full Robot',
            'ros2',
            launch_args,
        )
        self._update_tab_states_for_full_control()

    def send_all_status_commands(self, status_text=None):
        """Send all status commands sequentially"""
        if status_text is None:
            status_text = self.status_text
        status_commands = [
            'robotmode',
            'safetystatus',
            'programState',
            'running',
            'get loaded program',
            'is in remote control'
        ]
        
        self._log_append(status_text, "=" * 50)
        self._log_append(status_text, "📋 Sending all status commands...")
        self._log_append(status_text, "=" * 50)

        success_count = 0
        for command in status_commands:
            response = self._send_robot_command(command, status_text=status_text)
            if response is not None:
                success_count += 1
            # Add a small visual separator between commands
            self._log_append(status_text, "-" * 50)

        if success_count == len(status_commands):
            self._log_append(status_text, "✓ All status commands sent")
        elif success_count == 0:
            self._log_append(status_text, "✗ All status commands failed")
        else:
            self._log_append(status_text,
                f"⚠ {success_count}/{len(status_commands)} status commands succeeded"
            )
        self._log_append(status_text, "")

    def toggle_rqt_controller(self):
        self._toggle_process('rqt_controller', self.btn_rqt_controller, 'RQT Joint Controller',
                            'ros2', ['run', 'rqt_joint_trajectory_controller', 'rqt_joint_trajectory_controller', 
                                    '--force-discover'])
 
    def toggle_arduino_sensors(self):
        sim_mode = self.arm_sim_mode_combo.currentText()
        if sim_mode == 'true':
            self._toggle_process('arduino_sensors_sim', self.btn_arduino_sensors, 'Arduino Sensors (Sim Mode)',
                                'ros2', ['run', 'arm_control', 'arduino_sensors_sim'])
        else:
            self._toggle_process('arduino_sensors', self.btn_arduino_sensors, 'Arduino Sensors',
                            'ros2', ['run', 'arm_control', 'arduino_sensors'])
 
    def toggle_align_ee_to_wall(self):
        self._toggle_process('align_ee_to_wall', self.btn_align_ee_to_wall, 'Align EE to Wall',
                            'ros2', ['run', 'arm_control', 'align_ee_to_wall'])
        
    def toggle_mapping(self, mode='base', button=None, sim_combo=None, controller_type_combo=None, headless_combo=None, hybrid_sim_combo=None):
        """Toggle mapping with configurable mode parameter"""
        if button is None:
            button = self.btn_launch_mapping
        if sim_combo is None:
            sim_combo = self.sim_mode_combo
        if controller_type_combo is None:
            controller_type_combo = self.controller_type_combo
        if headless_combo is None:
            headless_combo = self.base_headless_combo if mode == 'base' else self.full_control_headless_combo
        if hybrid_sim_combo is None and mode == 'full' and hasattr(self, "full_control_hybrid_sim_combo"):
            hybrid_sim_combo = self.full_control_hybrid_sim_combo
        
        sim_mode = sim_combo.currentText()
        controller_type = controller_type_combo.currentText()
        headless = headless_combo.currentText() if headless_combo else 'false'
        if mode == 'full':
            hybrid_sim = self._sync_full_control_hybrid_sim_state()
        else:
            hybrid_sim = hybrid_sim_combo.currentText() if hybrid_sim_combo else 'false'
        
        args = [
            'launch',
            'navi_wall',
            'mapping_3d.launch.py',
            f'sim:={sim_mode}',
            f'mode:={mode}',
            f'controller_type:={controller_type}',
        ]
        if mode == 'full':
            args.append(f'hybrid_sim:={hybrid_sim}')
        args.append(f'headless:={headless}')
        
        process_key = f'{mode}_mapping' if mode != 'base' else 'mapping'
        display_name = 'Mapping'
        
        self._toggle_base_process(process_key, button, display_name, 'ros2', args)
        
        if mode == 'full':
            self._update_tab_states_for_full_control()  
        # Disable/enable localization button based on mapping state
        localization_button = self._get_localization_button_for_mode(mode)
        
        if localization_button:
            if process_key in self.process_map:
                localization_button.setEnabled(False)
            else:
                localization_button.setEnabled(True)
        
        # Update tab states only for base mode
        if mode == 'base':
            self._update_tab_states_for_base()

    
    def toggle_localization(self, mode='base', button=None, sim_combo=None, controller_type_combo=None, headless_combo=None, hybrid_sim_combo=None):
        """Toggle localization with configurable mode parameter"""
        if button is None:
            button = self.btn_launch_localization
        if sim_combo is None:
            sim_combo = self.sim_mode_combo
        if controller_type_combo is None:
            controller_type_combo = self.controller_type_combo
        if headless_combo is None:
            headless_combo = self.base_headless_combo if mode == 'base' else self.full_control_headless_combo
        if hybrid_sim_combo is None and mode == 'full' and hasattr(self, "full_control_hybrid_sim_combo"):
            hybrid_sim_combo = self.full_control_hybrid_sim_combo
        
        sim_mode = sim_combo.currentText()
        controller_type = controller_type_combo.currentText()
        headless = headless_combo.currentText() if headless_combo else 'false'
        if mode == 'full':
            hybrid_sim = self._sync_full_control_hybrid_sim_state()
        else:
            hybrid_sim = hybrid_sim_combo.currentText() if hybrid_sim_combo else 'false'
        
        process_key = f'{mode}_localization' if mode != 'base' else 'localization'
        display_name = 'Localization'
        
        localization_args = [
            'launch', 'navi_wall', 'move_robot.launch.py',
            f'sim:={sim_mode}', f'mode:={mode}', f'controller_type:={controller_type}',
        ]
        if mode == 'full':
            localization_args.append(f'hybrid_sim:={hybrid_sim}')
            localization_args.append(f'planner_backend:={self._get_planner_backend(context="full")}')
            localization_args.extend(self._get_moveit_launch_args(context='full'))
        localization_args.append(f'headless:={headless}')

        self._toggle_base_process(process_key, button, display_name, 'ros2', localization_args)
        if mode == 'full':
            self._update_tab_states_for_full_control()
        
        # Disable/enable mapping button based on localization state
        mapping_button = self._get_mapping_button_for_mode(mode)
        disable_buttons = self._get_buttons_to_disable_for_localization(mode)
        if mapping_button:
            if process_key in self.process_map:
                mapping_button.setEnabled(False)
                # Also disable other buttons that conflict with localization
                for btn in disable_buttons:
                    btn.setEnabled(False)
            else:
                mapping_button.setEnabled(True)
                # Re-enable other buttons when localization stops
                for btn in disable_buttons:
                    btn.setEnabled(True)
        
        # Update tab states only for base mode
        if mode == 'base':
            self._update_tab_states_for_base()

 
    def toggle_view_map(self, mode='base', button=None):
        if button is None:
            button = self.btn_view_map

        # Get package path for rtabmap.db
        try:
            pkg_share = get_package_share_directory('navi_wall')
            db_path = os.path.join(pkg_share, 'maps', 'rtabmap.db')
        except Exception as e:
            status_text = self.full_control_status_text if mode == 'full' else self.base_status_text
            self._log_append(status_text, f"Error: Could not find navi_wall package: {e}")
            return
        
        process_key = 'full_view_map' if mode == 'full' else 'view_map'
        self._toggle_base_process(process_key, button, 'View map',
                                 'rtabmap-databaseViewer', [db_path])
    
    def toggle_nav2(self, mode='base', button=None, sim_combo=None, controller_type_combo=None):
        """Toggle Nav2 with configurable mode parameter"""
        if button is None:
            button = self.btn_launch_nav2
        if sim_combo is None:
            sim_combo = self.sim_mode_combo
        if controller_type_combo is None:
            controller_type_combo = self.controller_type_combo
        
        sim_mode = sim_combo.currentText()
        controller_type = controller_type_combo.currentText()
        process_key = f'{mode}_nav2' if mode != 'base' else 'nav2'
        display_name = 'Nav2'
        
        self._toggle_base_process(process_key, button, display_name,
                                'ros2', ['launch', 'navi_wall', 'navigation_launch.py',
                                        f'use_sim_time:={sim_mode}', f'controller_type:={controller_type}'])
        if mode == 'full':
            self._update_tab_states_for_full_control()
        else:
            self._update_tab_states_for_base()
            
    def toggle_exploration(self, mode='base', button=None, sim_combo=None):
        """Toggle exploration with configurable mode parameter"""
        if button is None:
            button = self.btn_launch_exploration
        
        # Get package path for explore_params.yaml
        try:
            pkg_share = get_package_share_directory('navi_wall')
            params_file = os.path.join(pkg_share, 'config', 'explore_params.yaml')
        except:
            params_file = '/home/zed/ros2_ws/src/navi-wall/config/explore_params.yaml'
        
        process_key = f'{mode}_exploration' if mode != 'base' else 'exploration'
        display_name = 'Exploration'
        
        self._toggle_base_process(process_key, button, display_name,
                                'ros2', ['run', 'navi_wall', 'explore',
                                        '--ros-args', '--params-file', params_file])
        if mode == 'full':
            self._update_tab_states_for_full_control()
        else:
            self._update_tab_states_for_base()
   
    # Helper methods to get the correct buttons for each mode
    def _get_mapping_button_for_mode(self, mode):
        """Get the mapping button for a given mode"""
        if mode == 'base':
            return self.btn_launch_mapping
        elif mode == 'full':
            return self.btn_full_control_mapping
        return None
    def _get_buttons_to_disable_for_localization(self, mode):
        if mode == 'base':
            return [self.btn_launch_mapping, self.btn_launch_exploration, self.btn_launch_nav2]
        elif mode == 'full':
            return [self.btn_full_control_mapping, self.btn_full_control_exploration, self.btn_full_control_nav2]
        return []
    
    def _get_localization_button_for_mode(self, mode):
        """Get the localization button for a given mode"""
        if mode == 'base':
            return self.btn_launch_localization
        elif mode == 'full':
            return self.btn_full_control_localization
        return None

    def clear_full_control_status(self):
        widget = self.full_control_status_text
        self.tab_log_backups[id(widget)] = list(self.tab_log_entries.get(id(widget), []))
        self.tab_log_entries[id(widget)] = []
        widget.clear()
        self.btn_restore_full_control_status.setVisible(True)

    def restore_full_control_status(self):
        widget = self.full_control_status_text
        backup = self.tab_log_backups.get(id(widget))
        if backup:
            current = self.tab_log_entries.get(id(widget), [])
            self.tab_log_entries[id(widget)] = backup + current
            self.tab_log_backups[id(widget)] = []
            self._log_apply_filter(widget)
            self.btn_restore_full_control_status.setVisible(False)

    def toggle_rqt(self, mode='base', button=None):
        """Toggle RQT on/off"""
        if button is None:
            button = self.btn_launch_rqt
        process_key = 'full_rqt' if mode == 'full' else 'rqt'
        self._toggle_base_process(process_key, button, 'RQT', 'rqt', [])
 
    def run_ps_ros(self):
        """Run ps aux | grep ros2 command"""
        process = QProcess(self)
        process.setProcessChannelMode(QProcess.MergedChannels)
        process.readyReadStandardOutput.connect(lambda: self.handle_base_output(process))
 
        # Display command in bold green
        cmd_str = 'ps aux | grep -E \'ros2|robot\' | grep -v grep'
        self._log_append(self.base_status_text, f"<b style='color: #57ab5a;'>▶ {cmd_str}</b>")

        # Run the command using shell to support pipe
        process_key = 'ps_ros2'
        process.finished.connect(lambda: self._cleanup_ps_ros(process_key))
        process.start('bash', ['-c', 'ps aux | grep -E \'ros2|robot\' | grep -v grep'])
        self.process_map[process_key] = process
 
    def _cleanup_ps_ros(self, process_key):
        """Clean up finished ps aux process"""
        if process_key in self.process_map:
            del self.process_map[process_key]

    def _build_list_all_controllers_script(self):
        """Shell script that lists controllers for every detected controller_manager node."""
        return (
            "manager_nodes=$(ros2 node list 2>/dev/null | grep -E '(^|/)controller_manager$' | sort -u); "
            "if [ -z \"$manager_nodes\" ]; then "
            "  echo \"No controller_manager nodes found.\"; "
            "  exit 1; "
            "fi; "
            "for cm in $manager_nodes; do "
            "  echo \"===== $cm =====\"; "
            "  timeout 8 ros2 control list_controllers -c \"$cm\" || echo \"[WARN] Failed to query $cm\"; "
            "  echo; "
            "done"
        )
 
    def run_list_base_controllers(self):
        """List controllers for all detected controller_manager nodes."""
        process = QProcess(self)
        process.setProcessChannelMode(QProcess.MergedChannels)
        process.readyReadStandardOutput.connect(lambda: self.handle_base_output(process))
 
        # Display command in bold green
        cmd_str = "for cm in $(ros2 node list | grep -E '(^|/)controller_manager$'); do ros2 control list_controllers -c $cm; done"
        self._log_append(self.base_status_text, f"<b style='color: #57ab5a;'>▶ {cmd_str}</b>")

        # Run the command with timeout (10 seconds)
        process_key = 'list_base_controllers'
        process.finished.connect(lambda: self._cleanup_list_base_controllers(process_key))
        process.start('bash', ['-c', self._build_list_all_controllers_script()])
        self.process_map[process_key] = process
 
    def _cleanup_list_base_controllers(self, process_key):
        """Clean up finished list controllers process"""
        if process_key in self.process_map:
            exit_code = self.process_map[process_key].exitCode()
            del self.process_map[process_key]
            if exit_code != 0:
                self._log_append(self.base_status_text, f"<span style='color: #c69026;'>⚠ List controllers command finished with exit code {exit_code}</span>")
            else:
                self._log_append(self.base_status_text, "✓ List controllers command completed")

    def run_list_full_controllers(self):
        """List controllers for all detected controller_manager nodes (Full Control tab)."""
        process = QProcess(self)
        process.setProcessChannelMode(QProcess.MergedChannels)
        process.readyReadStandardOutput.connect(lambda: self.handle_full_control_output(process))

        # Display command in bold green
        cmd_str = "for cm in $(ros2 node list | grep -E '(^|/)controller_manager$'); do ros2 control list_controllers -c $cm; done"
        self._log_append(self.full_control_status_text, f"<b style='color: #57ab5a;'>▶ {cmd_str}</b>")

        # Run the command with timeout (10 seconds)
        process_key = 'full_list_base_controllers'
        process.finished.connect(lambda: self._cleanup_list_full_controllers(process_key))
        process.start('bash', ['-c', self._build_list_all_controllers_script()])
        self.process_map[process_key] = process

    def _cleanup_list_full_controllers(self, process_key):
        """Clean up finished list controllers process (Full Control tab)"""
        if process_key in self.process_map:
            exit_code = self.process_map[process_key].exitCode()
            del self.process_map[process_key]
            if exit_code != 0:
                self._log_append(self.full_control_status_text,
                    f"<span style='color: #c69026;'>⚠ List controllers command finished with exit code {exit_code}</span>"
                )
            else:
                self._log_append(self.full_control_status_text, "✓ List controllers command completed")
 
    def refresh_topics_list(self):
        """Refresh the list of available ROS2 topics"""
        process = QProcess(self)
        process.setProcessChannelMode(QProcess.MergedChannels)
        
        # Display command in status
        cmd_str = 'ros2 topic list'
        self._log_append(self.full_control_status_text, f"<b style='color: #57ab5a;'>▶ {cmd_str}</b>")

        # Start process and capture output
        process.finished.connect(lambda: self._on_topics_list_finished(process))
        process.start('ros2', ['topic', 'list'])
        
    def _on_topics_list_finished(self, process):
        """Process the output of ros2 topic list command"""
        output = process.readAllStandardOutput().data().decode('utf-8')
        
        # Clear and populate the combobox
        self.topics_combo.clear()
        topics = [line.strip() for line in output.split('\n') if line.strip()]
        
        if topics:
            self.topics_combo.addItems(topics)
            self._log_append(self.full_control_status_text, f"✓ Found {len(topics)} topics")
        else:
            self._log_append(self.full_control_status_text, "<span style='color: #c69026;'>⚠ No topics found</span>")
    
    def check_topic_bandwidth(self):
        """Check bandwidth of selected topic"""
        topic = self.topics_combo.currentText()
        if not topic:
            self.topic_info_display.setPlainText("⚠ No topic selected")
            return
        
        self.topic_info_display.setPlainText("Measuring bandwidth...")
        process = QProcess(self)
        process.setProcessChannelMode(QProcess.MergedChannels)
        
        # Store flags and buffers for this process
        self._bw_output_buffer = ""
        self._bw_result_found = False
        process.readyReadStandardOutput.connect(lambda: self._accumulate_bw_output(process))
        
        # Start process with timeout
        process_key = 'topic_bw'
        process.finished.connect(lambda: self._on_bandwidth_finished(process))
        process.start('timeout', ['4', 'ros2', 'topic', 'bw', topic])
        self.process_map[process_key] = process
    
    def _accumulate_bw_output(self, process):
        """Accumulate bandwidth output as it arrives and parse first result"""
        output = process.readAllStandardOutput().data().decode('utf-8')
        self._bw_output_buffer += output
        
        # If we haven't found a result yet, try to parse it now
        if not self._bw_result_found:
            lines = self._bw_output_buffer.split('\n')
            for line in lines:
                if 'Message size mean:' in line:
                    try:
                        parts = line.split('Message size mean:')
                        if len(parts) > 1:
                            mean_part = parts[1].strip().split()[0:2]  # Get "0.07 MB"
                            msg_size_mean = ' '.join(mean_part)
                            self.topic_info_display.setPlainText(f"Message size mean: {msg_size_mean}")
                            self._bw_result_found = True
                            # Kill the process since we got what we need
                            if 'topic_bw' in self.process_map:
                                self.process_map['topic_bw'].kill()
                            break
                    except:
                        pass
    
    def check_topic_frequency(self):
        """Check frequency of selected topic"""
        topic = self.topics_combo.currentText()
        if not topic:
            self.topic_info_display.setPlainText("⚠ No topic selected")
            return
        
        self.topic_info_display.setPlainText("Measuring frequency...")
        process = QProcess(self)
        process.setProcessChannelMode(QProcess.MergedChannels)
        
        # Store flags and buffers for this process
        self._hz_output_buffer = ""
        self._hz_result_found = False
        process.readyReadStandardOutput.connect(lambda: self._accumulate_hz_output(process))
        
        # Start process with timeout
        process_key = 'topic_hz'
        process.finished.connect(lambda: self._on_frequency_finished(process))
        process.start('timeout', ['4', 'ros2', 'topic', 'hz', topic])
        self.process_map[process_key] = process
    
    def _accumulate_hz_output(self, process):
        """Accumulate frequency output as it arrives and parse first result"""
        output = process.readAllStandardOutput().data().decode('utf-8')
        self._hz_output_buffer += output
        
        # If we haven't found a result yet, try to parse it now
        if not self._hz_result_found:
            lines = self._hz_output_buffer.split('\n')
            for line in lines:
                if 'average rate:' in line:
                    try:
                        parts = line.split('average rate:')
                        if len(parts) > 1:
                            avg_rate = parts[1].strip().split()[0]
                            self.topic_info_display.setPlainText(f"Average rate: {avg_rate} Hz")
                            self._hz_result_found = True
                            # Kill the process since we got what we need
                            if 'topic_hz' in self.process_map:
                                self.process_map['topic_hz'].kill()
                            break
                    except:
                        pass
    
    def echo_topic_once(self):
        """Echo selected topic once"""
        topic = self.topics_combo.currentText()
        if not topic:
            self._log_append(self.full_control_status_text, "<span style='color: #c69026;'>⚠ No topic selected</span>")
            return
        
        process = QProcess(self)
        process.setProcessChannelMode(QProcess.MergedChannels)
        process.readyReadStandardOutput.connect(lambda: self.handle_full_control_output(process))
        
        # Display command
        cmd_str = f'timeout 10 ros2 topic echo --once {topic}'
        self._log_append(self.full_control_status_text, f"<b style='color: #57ab5a;'>▶ {cmd_str}</b>")

        # Start process with timeout
        process_key = f'topic_echo_{topic}'
        process.finished.connect(lambda: self._cleanup_topic_command(process_key))
        process.start('timeout', ['10', 'ros2', 'topic', 'echo', '--once', topic])
        self.process_map[process_key] = process
    
    def _on_bandwidth_finished(self, process):
        """Handle bandwidth process completion"""
        if 'topic_bw' in self.process_map:
            del self.process_map['topic_bw']
        
        # If we didn't get a result during execution, try one more time
        if not self._bw_result_found:
            final_output = process.readAllStandardOutput().data().decode('utf-8')
            output = self._bw_output_buffer + final_output
            
            if not output.strip():
                self.topic_info_display.setPlainText("⚠ No data received")
                return
            
            # Try to find the result in the complete output
            lines = output.split('\n')
            for line in lines:
                if 'Message size mean:' in line:
                    try:
                        parts = line.split('Message size mean:')
                        if len(parts) > 1:
                            mean_part = parts[1].strip().split()[0:2]
                            msg_size_mean = ' '.join(mean_part)
                            self.topic_info_display.setPlainText(f"Message size mean: {msg_size_mean}")
                            return
                    except:
                        pass
            
            self.topic_info_display.setPlainText("⚠ Could not parse bandwidth data")
        
        # Clear buffer
        self._bw_output_buffer = ""
    
    def _on_frequency_finished(self, process):
        """Handle frequency process completion"""
        if 'topic_hz' in self.process_map:
            del self.process_map['topic_hz']
        
        # If we didn't get a result during execution, try one more time
        if not self._hz_result_found:
            final_output = process.readAllStandardOutput().data().decode('utf-8')
            output = self._hz_output_buffer + final_output
            
            if not output.strip():
                self.topic_info_display.setPlainText("⚠ No data received")
                return
            
            # Try to find the result in the complete output
            lines = output.split('\n')
            for line in lines:
                if 'average rate:' in line:
                    try:
                        parts = line.split('average rate:')
                        if len(parts) > 1:
                            avg_rate = parts[1].strip().split()[0]
                            self.topic_info_display.setPlainText(f"Average rate: {avg_rate} Hz")
                            return
                    except:
                        pass
            
            self.topic_info_display.setPlainText("⚠ Could not parse frequency data")
        
        # Clear buffer
        self._hz_output_buffer = ""
    
    def _cleanup_topic_command(self, process_key):
        """Clean up finished topic command process"""
        if process_key in self.process_map:
            exit_code = self.process_map[process_key].exitCode()
            del self.process_map[process_key]
            if exit_code == 124:  # timeout exit code
                self._log_append(self.full_control_status_text, "<span style='color: #c69026;'>⚠ Command timed out</span>")
            elif exit_code != 0:
                self._log_append(self.full_control_status_text, f"<span style='color: #c69026;'>⚠ Command finished with exit code {exit_code}</span>")
            else:
                self._log_append(self.full_control_status_text, "✓ Command completed")

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
            if process_key in ['ur_control', 'arm_launch']:
                pid = process.processId()
                if pid:
                    try:
                        # Kill all children of this specific launch process
                        subprocess.run(['pkill', '-9', '-P', str(pid)], timeout=2, stderr=subprocess.DEVNULL)
                    except:
                        pass
                # Extra cleanup for child ROS processes spawned by launch files
                self._cleanup_ros_children_of_pid(pid)
                # Kill Gazebo processes when stopping Arm launch
                if process_key == 'arm_launch':
                    self._kill_gazebo_processes()
 
            process.terminate()
            process.waitForFinished(3000)
            if process.state() == QProcess.Running:
                process.kill()
 
            del self.process_map[process_key]
            button.setText(f"Start {name}")
            button.setStyleSheet("")
            self._log_append(self.status_text, f"⏹ Stopped {name}")
        else:
            # Start the process
            process = QProcess(self)
            process.setProcessChannelMode(QProcess.MergedChannels)
            process.readyReadStandardOutput.connect(lambda: self.handle_output(process))
            process.finished.connect(lambda: self._on_process_finished(process_key, button, name))

            # Display command in bold green
            cmd_str = program + ' ' + ' '.join(args)
            self._log_append(self.status_text, f"<b style='color: #57ab5a;'>▶ {cmd_str}</b>")
 
            process.start(program, args)
            self.process_map[process_key] = process
            button.setText(f"Stop {name}")
            button.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;")
 
    def _toggle_base_process(self, process_key, button, name, program, args):
        # Decide status widget
        status_text = self.full_control_status_text if process_key.startswith('full') else self.base_status_text

        if process_key in self.process_map:
            # ===== STOP PROCESS =====
            process = self.process_map[process_key]

            # Disconnect finished to avoid double cleanup
            try:
                process.finished.disconnect()
            except Exception:
                pass

            pid = process.processId()

            # Try graceful SIGINT (Ctrl+C equivalent)
            if pid:
                try:
                    os.kill(pid, signal.SIGINT)
                    # Longer wait for mapping to save rtabmap.db
                    wait_ms = 5000 if 'mapping' in process_key else 3000
                    if 'mapping' in process_key:
                        self._log_append(status_text, "💾 Saving mapping database... (waiting for shutdown)")
                    process.waitForFinished(wait_ms)
                except ProcessLookupError:
                    # Process already gone
                    pass
                except Exception as e:
                    self._log_append(status_text, f"⚠ Could not send SIGINT: {e}")

            # If still running, escalate
            if process.state() == QProcess.Running:
                process.terminate()
                process.waitForFinished(2000)
            if process.state() == QProcess.Running:
                process.kill()
                process.waitForFinished(2000)

            # Kill orphaned launch children/Gazebo for simulator-backed launches.
            if self._is_robot_bringup_process(process_key) and pid:
                try:
                    subprocess.run(['pkill', '-9', '-P', str(pid)], timeout=2, stderr=subprocess.DEVNULL)
                except Exception:
                    pass
                self._cleanup_ros_children_of_pid(pid)

            uses_gazebo = bool(button.property('uses_gazebo'))

            # Kill Gazebo processes when stopping mapping, localization, or a sim-backed full robot bringup
            if (
                'mapping' in process_key
                or 'localization' in process_key
                or (self._is_robot_bringup_process(process_key) and uses_gazebo)
            ):
                self._kill_gazebo_processes()

            # Final cleanup
            if process_key in self.process_map:
                del self.process_map[process_key]

            button.setText(self._get_base_process_start_text(process_key, name))
            button.setStyleSheet("")
            if self._is_robot_bringup_process(process_key):
                button.setProperty('uses_gazebo', False)
            self._log_append(status_text, f"⏹ Stopped {name}")

            # Re‑enable mutually exclusive buttons
            if 'mapping' in process_key:
                mode = 'full' if process_key.startswith('full') else 'base'
                btn = self._get_localization_button_for_mode(mode)
                if btn:
                    btn.setEnabled(True)
            elif 'localization' in process_key:
                mode = 'full' if process_key.startswith('full') else 'base'
                btn = self._get_mapping_button_for_mode(mode)
                if btn:
                    btn.setEnabled(True)

            # Only base mode affects tab states
            if not process_key.startswith('full') and self._is_base_tab_state_process(process_key):
                self._update_tab_states_for_base()

        else:
            # ===== START PROCESS (unchanged) =====
            process = QProcess(self)
            process.setProcessChannelMode(QProcess.MergedChannels)

            if process_key.startswith('full'):
                process.readyReadStandardOutput.connect(lambda: self.handle_full_control_output(process))
            else:
                process.readyReadStandardOutput.connect(lambda: self.handle_base_output(process))

            process.finished.connect(lambda: self._on_base_process_finished(process_key, button, name))

            cmd_str = program + ' ' + ' '.join(args)
            self._log_append(status_text, f"<b style='color:#57ab5a;'>▶ {cmd_str}</b>")

            process.start(program, args)
            self.process_map[process_key] = process
            button.setText(self._get_base_process_stop_text(process_key, name))
            button.setStyleSheet("background-color:#4CAF50; color:white; font-weight:bold;")


            
    def handle_full_control_output(self, process):
        """Handle output for full control processes (outputs to full_control_status_text)"""
        output = process.readAllStandardOutput().data().decode()
        if output:
            lines = output.split('\n')
            for line in lines:
                # Skip expected shutdown messages
                if 'process has died' in line and 'exit code -9' in line:
                    continue

                # Convert ANSI color codes to HTML
                html_line = self._ansi_to_html(line)

                # Use insertHtml to properly render HTML entities
                self._log_append(self.full_control_status_text, html_line)

    def handle_joint_output(self, process):
        """Handle output for joint control processes (outputs to joint_status_text)"""
        output = process.readAllStandardOutput().data().decode()
        if output:
            lines = output.split('\n')
            for line in lines:
                # Skip expected shutdown messages
                if 'process has died' in line and 'exit code -9' in line:
                    continue

                # Convert ANSI color codes to HTML
                html_line = self._ansi_to_html(line)

                # Use insertHtml to properly render HTML entities
                self._append_to_text_widget(self.joint_status_text, html_line)
 
    def _on_process_finished(self, process_key, button, name):
        """Handle when a process finishes unexpectedly"""
        if process_key in self.process_map:
            del self.process_map[process_key]
            button.setText(f"Start {name}")
            button.setStyleSheet("")
            self._log_append(self.status_text, f"⚠ {name} exited")
            
            # Update tab states when arm processes finish
            if process_key == 'arm_launch':
                self._update_tab_states_for_arm()

 
    def _on_base_process_finished(self, process_key, button, name):
        """Handle when a base process finishes unexpectedly"""
        if process_key in self.process_map:
            del self.process_map[process_key]
            button.setText(self._get_base_process_start_text(process_key, name))
            button.setStyleSheet("")
            
            # Determine which status text to use
            if process_key.startswith('full'):
                status_text = self.full_control_status_text
            else:
                status_text = self.base_status_text
            
            self._log_append(status_text, f"{name} exited")

            if self._is_robot_bringup_process(process_key) and bool(button.property('uses_gazebo')):
                self._kill_gazebo_processes()
                button.setProperty('uses_gazebo', False)
            
            # Re-enable mutually exclusive buttons
            if 'mapping' in process_key:
                mode = 'full' if process_key.startswith('full') else 'base'
                localization_button = self._get_localization_button_for_mode(mode)
                if localization_button:
                    localization_button.setEnabled(True)
            elif 'localization' in process_key:
                mode = 'full' if process_key.startswith('full') else 'base'
                mapping_button = self._get_mapping_button_for_mode(mode)
                if mapping_button:
                    mapping_button.setEnabled(True)
                # Re-enable other buttons disabled during localization
                for btn in self._get_buttons_to_disable_for_localization(mode):
                    btn.setEnabled(True)
            
            # Update tab states when base processes finish
            if not process_key.startswith('full') and self._is_base_tab_state_process(process_key):
                self._update_tab_states_for_base()

            elif process_key.startswith('full') and any(
                p in process_key
                for p in ['mobile_manipulator', 'mapping', 'localization', 'nav2', 'exploration']
            ):
                self._update_tab_states_for_full_control()
 
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
                self._log_append(self.status_text, html_line)

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
                self._log_append(self.base_status_text, html_line)

    def _connect_robot_socket(self, status_text=None):
        """Connect to robot dashboard if not already connected"""
        if status_text is None:
            status_text = self.status_text

        context = 'full' if status_text is self.full_control_status_text else 'arm'
        desired_host = self._get_robot_ip_for_launch(context=context)
        if self.robot_host != desired_host:
            if self.robot_socket:
                try:
                    self.robot_socket.close()
                except Exception:
                    pass
                self.robot_socket = None
            self.robot_host = desired_host

        if self.robot_socket is None:
            try:
                self.robot_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.robot_socket.settimeout(5)
                self.robot_socket.connect((self.robot_host, self.robot_port))
                # Read initial connection message
                data = self.robot_socket.recv(1024)
                response = data.decode('utf-8').strip()
                self._log_append(status_text, f"✓ Connected to robot: {response}")
                return True
            except Exception as e:
                self._log_append(status_text, f"<span style='color: #f47067;'>✗ Failed to connect to robot: {e}</span>")
                self.robot_socket = None
                return False
        return True
 
    def _build_status_indicators(self, parent_layout):
        """Add Robot Mode / Safety Status / Program State indicator rows to parent_layout.

        Returns a dict keyed by command name ('robotmode', 'safetystatus',
        'programState') with (icon_label, value_label) tuples for later updates.
        """
        indicators = {}
        rows = [
            ('robotmode',    'Robot Mode'),
            ('safetystatus', 'Safety Status'),
            ('programState', 'Program State'),
        ]
        for key, label_text in rows:
            row = QHBoxLayout()
            icon = QLabel("○")
            icon.setStyleSheet("color: #8b949e; font-size: 16pt; font-weight: bold;")
            icon.setFixedWidth(24)
            icon.setAlignment(Qt.AlignCenter)
            name = QLabel(label_text + ":")
            name.setStyleSheet("font-weight: bold;")
            name.setFixedWidth(110)
            value = QLabel("—")
            value.setStyleSheet("color: #8b949e;")
            value.setWordWrap(True)
            row.addWidget(icon)
            row.addWidget(name)
            row.addWidget(value, 1)
            parent_layout.addLayout(row)
            indicators[key] = (icon, value)
        return indicators

    def _update_status_indicator(self, command, response, status_text):
        """Update icon/value labels based on the dashboard response."""
        if command not in ('robotmode', 'safetystatus', 'programState'):
            return
        if status_text is self.status_text:
            indicators = getattr(self, 'arm_status_indicators', None)
        elif status_text is getattr(self, 'full_control_status_text', None):
            indicators = getattr(self, 'full_status_indicators', None)
        else:
            indicators = None
        if not indicators or command not in indicators:
            return

        icon_label, value_label = indicators[command]
        text = (response or "").strip()
        GREEN = "#2ea043"
        RED = "#f47067"
        GREY = "#8b949e"

        if command == 'robotmode':
            val = text.split(':', 1)[-1].strip() if ':' in text else text
            ok = val.upper() == 'RUNNING'
            icon_label.setText("●")
            icon_label.setStyleSheet(
                f"color: {GREEN if ok else RED}; font-size: 16pt; font-weight: bold;"
            )
            value_label.setText(val or "—")
            value_label.setStyleSheet(f"color: {GREEN if ok else RED};")
        elif command == 'safetystatus':
            val = text.split(':', 1)[-1].strip() if ':' in text else text
            ok = val.upper() == 'NORMAL'
            icon_label.setText("☑" if ok else "⛔")
            icon_label.setStyleSheet(
                f"color: {GREEN if ok else RED}; font-size: 16pt; font-weight: bold;"
            )
            value_label.setText(val or "—")
            value_label.setStyleSheet(f"color: {GREEN if ok else RED};")
        elif command == 'programState':
            first = text.split()[0].upper() if text else ""
            ok = first == 'PLAYING'
            icon_label.setText("●")
            icon_label.setStyleSheet(
                f"color: {GREEN if ok else RED}; font-size: 16pt; font-weight: bold;"
            )
            value_label.setText("PLAYING" if ok else "STOPPED")
            value_label.setStyleSheet(f"color: {GREEN if ok else RED};")

    # Map of control commands to the status query(ies) that should be re-issued
    # afterwards so the indicators reflect the actual post-command robot state.
    _CONTROL_STATUS_REFRESH = {
        'power on':                 ['robotmode'],
        'power off':                ['robotmode'],
        'brake release':            ['robotmode'],
        'shutdown':                 ['robotmode'],
        'play':                     ['programState', 'robotmode'],
        'pause':                    ['programState'],
        'stop':                     ['programState'],
        'load Test_external_control.urp': ['programState'],
        'restart safety':           ['safetystatus', 'robotmode'],
        'close safety popup':       ['safetystatus'],
        'unlock protective stop':   ['safetystatus', 'robotmode'],
    }

    def _schedule_status_refresh(self, command, status_text):
        """If `command` is a control command we know about, query the related
        status(es) shortly afterwards so the indicators reflect the actual
        robot state (e.g. 'play' that fails should not leave PLAYING showing)."""
        refreshes = self._CONTROL_STATUS_REFRESH.get(command)
        if not refreshes:
            return

        def _do_refresh():
            for s in refreshes:
                self._send_robot_command(s, status_text=status_text)

        # Small delay so the robot has time to transition before we query.
        QTimer.singleShot(500, _do_refresh)

    def _send_robot_command(self, command, status_text=None):
        """Send command to robot dashboard and return response"""
        if status_text is None:
            status_text = self.status_text
        if not self._connect_robot_socket(status_text=status_text):
            return None

        try:
            self.robot_socket.send(str.encode(command + '\n'))
            self._log_append(status_text, f"<b style='color: #57ab5a;'>→ SENT: {command}</b>")
            self._log_append(status_text, "")  # Add newline after command

            data = self.robot_socket.recv(1024)
            response = data.decode('utf-8').strip()
            self._log_append(status_text, f"← RECV: {response}")
            self._update_status_indicator(command, response, status_text)
            self._schedule_status_refresh(command, status_text)
            return response
        except Exception as e:
            self._log_append(status_text, f"<span style='color: #f47067;'>✗ Command failed: {e}</span>")
            # Close socket on error so it reconnects next time
            if self.robot_socket:
                self.robot_socket.close()
                self.robot_socket = None
            return None
 
    def send_control_command(self, status_text=None):
        """Send selected control command to robot"""
        command = self.control_cmd_combo.currentText()
        self._send_robot_command(command, status_text=status_text)

    def send_all_full_control_status_commands(self):
        """Send all status commands sequentially (Full Control tab)"""
        status_commands = [
            'robotmode',
            'safetystatus',
            'programState',
            'running',
            'get loaded program',
            'is in remote control'
        ]
        
        self._log_append(self.full_control_status_text, "=" * 50)
        self._log_append(self.full_control_status_text, "📋 Sending all status commands...")
        self._log_append(self.full_control_status_text, "=" * 50)

        success_count = 0
        for command in status_commands:
            response = self._send_robot_command(command, status_text=self.full_control_status_text)
            if response is not None:
                success_count += 1
            # Add a small visual separator between commands
            self._log_append(self.full_control_status_text, "-" * 50)

        if success_count == len(status_commands):
            self._log_append(self.full_control_status_text, "✓ All status commands sent")
        elif success_count == 0:
            self._log_append(self.full_control_status_text, "✗ All status commands failed")
        else:
            self._log_append(self.full_control_status_text,
                f"⚠ {success_count}/{len(status_commands)} status commands succeeded"
            )
        self._log_append(self.full_control_status_text, "")
    
    def send_full_control_control_command(self):
        """Send selected control command to robot (Full Control tab)"""
        command = self.full_control_control_cmd_combo.currentText()
        self._send_robot_command(command, status_text=self.full_control_status_text)
    
    def run_full_control_ps_ros(self):
        """Run ps aux | grep ros2 command (Full Control tab)"""
        process = QProcess(self)
        process.setProcessChannelMode(QProcess.MergedChannels)
        
        # Clear accumulator for new ps command
        self.ps_output_accumulator = ""
        
        # Connect to special handler that accumulates output
        process.readyReadStandardOutput.connect(lambda: self._handle_ps_output(process))
 
        # Display command in bold green
        cmd_str = 'ps aux | grep -E \'ros2|robot\' | grep -v grep'
        self._log_append(self.full_control_status_text, f"<b style='color: #57ab5a;'>▶ {cmd_str}</b>")

        # Run the command using shell to support pipe
        process_key = 'full_ps_ros2'
        process.finished.connect(lambda: self._cleanup_full_ps_ros(process_key, process))
        process.start('bash', ['-c', 'ps aux | grep -E \'ros2|robot\' | grep -v grep'])
        self.process_map[process_key] = process
 
    def _handle_ps_output(self, process):
        """Handle output from ps command, accumulating it and displaying it"""
        output = process.readAllStandardOutput().data().decode()
        if output:
            # Accumulate for later parsing
            self.ps_output_accumulator += output
            
            # Also display it
            lines = output.split('\n')
            for line in lines:
                if line.strip():
                    html_line = self._ansi_to_html(line)
                    self._log_append(self.full_control_status_text, html_line)

    def _cleanup_full_ps_ros(self, process_key, process):
        """Clean up finished ps aux process and populate process combobox (Full Control tab)"""
        # Read any remaining output
        remaining_output = process.readAllStandardOutput().data().decode()
        if remaining_output:
            self.ps_output_accumulator += remaining_output
        
        # Parse the accumulated process list and populate combobox
        self.populate_process_combo(self.ps_output_accumulator)
        
        if process_key in self.process_map:
            del self.process_map[process_key]
    
    def toggle_full_control_align_ee(self):
        """Toggle Align EE to Wall (Full Control tab)"""
        self._toggle_full_control_process('full_align_ee_to_wall', self.btn_full_control_align_ee, 
                                         'Align EE to Wall', 'ros2', 
                                         ['run', 'arm_control', 'align_ee_to_wall'])
    
    def _toggle_full_control_process(self, process_key, button, name, program, args):
        """Toggle a process on/off for Full Control tab"""
        if process_key in self.process_map:
            # Stop the process
            process = self.process_map[process_key]
            try:
                process.finished.disconnect()
            except:
                pass
 
            process.terminate()
            process.waitForFinished(3000)
            if process.state() == QProcess.Running:
                process.kill()
 
            del self.process_map[process_key]
            button.setText(f"Start {name}")
            button.setStyleSheet("")
            self._log_append(self.full_control_status_text, f"⏹ Stopped {name}")
        else:
            # Start the process
            process = QProcess(self)
            process.setProcessChannelMode(QProcess.MergedChannels)
            process.readyReadStandardOutput.connect(lambda: self.handle_full_control_output(process))
            process.finished.connect(lambda: self._on_full_control_process_finished(process_key, button, name))

            # Display command in bold green
            cmd_str = program + ' ' + ' '.join(args)
            self._log_append(self.full_control_status_text, f"<b style='color: #57ab5a;'>▶ {cmd_str}</b>")
 
            process.start(program, args)
            self.process_map[process_key] = process
            button.setText(f"Stop {name}")
            button.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;")
    
    def _on_full_control_process_finished(self, process_key, button, name):
        """Handle when a Full Control process finishes unexpectedly"""
        if process_key in self.process_map:
            del self.process_map[process_key]
            button.setText(f"Start {name}")
            button.setStyleSheet("")
            self._log_append(self.full_control_status_text, f"⚠ {name} exited")

    def list_controllers(self):
        """List controllers for all detected controller_manager nodes (Arm tab)."""
        process = QProcess(self)
        process.setProcessChannelMode(QProcess.MergedChannels)
        process.readyReadStandardOutput.connect(lambda: self.handle_output(process))
 
        # Display command in bold green
        cmd_str = "for cm in $(ros2 node list | grep -E '(^|/)controller_manager$'); do ros2 control list_controllers -c $cm; done"
        self._log_append(self.status_text, f"<b style='color: #57ab5a;'>▶ {cmd_str}</b>")

        # Run the command
        process_key = 'list_controllers'
        process.finished.connect(lambda: self._cleanup_list_controllers(process_key))
        process.start('bash', ['-c', self._build_list_all_controllers_script()])
        self.process_map[process_key] = process
 
    def _cleanup_list_controllers(self, process_key):
        """Clean up finished list controllers process"""
        if process_key in self.process_map:
            exit_code = self.process_map[process_key].exitCode()
            del self.process_map[process_key]
            if exit_code != 0:
                self._log_append(self.status_text, f"<span style='color: #c69026;'>⚠ List controllers command finished with exit code {exit_code}</span>")
            else:
                self._log_append(self.status_text, "✓ List controllers command completed")
 
    def clear_status(self):
        widget = self.status_text
        self.tab_log_backups[id(widget)] = list(self.tab_log_entries.get(id(widget), []))
        self.tab_log_entries[id(widget)] = []
        widget.clear()
        self.btn_restore_status.setVisible(True)

    def restore_status(self):
        widget = self.status_text
        backup = self.tab_log_backups.get(id(widget))
        if backup:
            current = self.tab_log_entries.get(id(widget), [])
            self.tab_log_entries[id(widget)] = backup + current
            self.tab_log_backups[id(widget)] = []
            self._log_apply_filter(widget)
            self.btn_restore_status.setVisible(False)

    def clear_base_status(self):
        widget = self.base_status_text
        self.tab_log_backups[id(widget)] = list(self.tab_log_entries.get(id(widget), []))
        self.tab_log_entries[id(widget)] = []
        widget.clear()
        self.btn_restore_base_status.setVisible(True)

    def restore_base_status(self):
        widget = self.base_status_text
        backup = self.tab_log_backups.get(id(widget))
        if backup:
            current = self.tab_log_entries.get(id(widget), [])
            self.tab_log_entries[id(widget)] = backup + current
            self.tab_log_backups[id(widget)] = []
            self._log_apply_filter(widget)
            self.btn_restore_base_status.setVisible(False)

    def clear_joint_status(self):
        """Clear joint control status text"""
        self.joint_status_text.clear()
    
    def read_joint_positions(self, silent=False):
        """Read current joint positions from the active joint_states topic and populate input fields."""
        process = QProcess(self)
        process.setProcessChannelMode(QProcess.MergedChannels)
        joint_states_topic = self._get_joint_states_topic_for_ui(context='full')
        
        # Display command in bold green (only if not silent)
        if not silent:
            self._append_to_text_widget(
                self.joint_status_text,
                f"<b style='color: #539bf5;'>▶ ros2 topic echo {joint_states_topic} --once</b><br>"
            )
        
        # Store the process to retrieve output later
        process.finished.connect(lambda: self.parse_joint_states(process, silent))
        process.start('bash', ['-c', f'timeout 5 ros2 topic echo {joint_states_topic} --once'])

    
    def parse_joint_states(self, process, silent=False):
        """Parse joint states output and populate input fields"""
        output = process.readAllStandardOutput().data().decode('utf-8')
        
        # Display the output line by line to preserve formatting (only if not silent)
        if not silent:
            lines = output.split('\n')
            for line in lines:
                html_line = self._ansi_to_html(line)
                self._append_to_text_widget(self.joint_status_text, html_line)
        
        # Check if topic actually published data (not timeout or error)
        if not output.strip() or 'ERROR' in output or output.strip().startswith('timeout'):
            self.joint_status_text.append("")
            self.joint_status_text.insertHtml("<span style='color: #f47067;'>✗ No data received - Topic may not be publishing yet</span><br>")
            self.joint_status_text.append("")
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
                
                if name_section and line.strip().startswith('-'):
                    joint_name = line.strip()[2:].strip()
                    joint_names.append(joint_name)
                elif position_section and line.strip().startswith('-'):
                    try:
                        position = float(line.strip()[2:].strip())
                        positions.append(position)
                    except ValueError:
                        pass
            
            # Verify we have valid data before enabling controls
            if len(joint_names) == 0 or len(positions) == 0:
                if not silent:
                    self.joint_status_text.append("")
                    self.joint_status_text.insertHtml("<span style='color: #f47067;'>✗ No valid joint data received</span><br>")
                    self.joint_status_text.append("")
                return
            
            if len(joint_names) != len(positions):
                if not silent:
                    self.joint_status_text.append("")
                    self.joint_status_text.insertHtml("<span style='color: #f47067;'>✗ Error: Joint names and positions count mismatch</span><br>")
                    self.joint_status_text.append("")
                return
            
            # Create a mapping from joint name to position
            joint_position_map = dict(zip(joint_names, positions))
            
            # Expected joint order (for UI display and publishing)
            expected_joints = self.arm_joint_names
            
            # Verify all expected joints are present
            missing_joints = [j for j in expected_joints if j not in joint_position_map]
            if missing_joints:
                if not silent:
                    self.joint_status_text.append("")
                    self.joint_status_text.insertHtml(f"<span style='color: #f47067;'>✗ Missing joints: {', '.join(missing_joints)}</span><br>")
                    self.joint_status_text.append("")
                return
            
            # Block signals to prevent cascading updates while setting values
            for input_field in self.joint_inputs:
                input_field.blockSignals(True)
            for slider in self.joint_sliders:
                slider.blockSignals(True)
            
            # Populate input fields in the expected order
            for i, joint_name in enumerate(expected_joints):
                if joint_name in joint_position_map:
                    self.joint_inputs[i].setValue(joint_position_map[joint_name])
                    # Also update slider to match
                    self.joint_sliders[i].setValue(int(joint_position_map[joint_name] * 100))
            
            # Unblock signals
            for input_field in self.joint_inputs:
                input_field.blockSignals(False)
            for slider in self.joint_sliders:
                slider.blockSignals(False)
            
            # Store current joint positions for safety comparison (as dictionary by joint name)
            self.current_joint_positions = {joint_name: joint_position_map[joint_name] for joint_name in expected_joints}
            
            # Enable controls only after confirming we have valid, complete joint data
            for input_field in self.joint_inputs:
                input_field.setEnabled(True)
            for slider in self.joint_sliders:
                slider.setEnabled(True)
            self.btn_publish_joints.setEnabled(True)
            
            # Success message
            self.joint_status_text.append("")
            if silent:
                self.joint_status_text.insertHtml("<span style='color: #76e3ea;'>🔄 Joint positions updated</span><br>")
            else:
                self.joint_status_text.insertHtml("<span style='color: #57ab5a;'>✓ Joint positions updated - Controls enabled</span><br>")
            self.joint_status_text.append("")
            
        except Exception as e:
            if not silent:
                self.joint_status_text.append("")
                self.joint_status_text.insertHtml(f"<span style='color: #f47067;'>✗ Error parsing joint states: {str(e)}</span><br>")
                self.joint_status_text.append("")

     
    def publish_joint_trajectory(self):
        """Publish joint trajectory to the active planned_trajectory topic."""
        planned_trajectory_topic = self._get_planned_trajectory_topic_for_ui()

        # Expected joint order for publishing
        expected_joints = self.arm_joint_names
        
        # Get joint positions from input fields (in expected order)
        positions = [field.value() for field in self.joint_inputs]
        positions_str = ', '.join([f'{pos:.4f}' for pos in positions])
        
        # Safety check: ensure only one joint position is different from current
        if hasattr(self, 'current_joint_positions') and self.current_joint_positions:
            differences = []
            tolerance = 0.015
            
            for i, (joint_name, requested) in enumerate(zip(expected_joints, positions)):
                if joint_name in self.current_joint_positions:
                    current = self.current_joint_positions[joint_name]
                    if abs(current - requested) > tolerance:
                        differences.append((i, joint_name, current, requested))
            
            if len(differences) > 1:
                changed_joints_str = ', '.join([f"{name} (Joint {i})" for i, name, _, _ in differences])
                self.joint_status_text.append("")
                self.joint_status_text.insertHtml("<b style='color: #f47067;'>⚠ Safety Warning: Cannot change multiple joints at once!</b><br>")
                self.joint_status_text.append(f"Changed joints: {changed_joints_str}")
                self.joint_status_text.append("Requested joint positions should differ in only one joint for safety.")
                self.joint_status_text.append("")
                
                # Show the differences for debugging
                self.joint_status_text.append("Differences detected:")
                for i, name, current, requested in differences:
                    diff = abs(requested - current)
                    self.joint_status_text.append(f"  {name}: Current={current:.4f}, Requested={requested:.4f}, Diff={diff:.4f} rad")
                self.joint_status_text.append("")
                return
            elif len(differences) == 0:
                self.joint_status_text.append("")
                self.joint_status_text.insertHtml("<b style='color: #c69026;'>⚠ Warning: No joint positions changed</b><br>")
                self.joint_status_text.append("Current and requested positions are identical (within tolerance).")
                self.joint_status_text.append("")
                return
            else:
                # Exactly one joint changed - show which one
                i, name, current, requested = differences[0]
                diff = abs(requested - current)
                self.joint_status_text.append("")
                self.joint_status_text.insertHtml(f"<b style='color: #57ab5a;'>✓ Moving single joint: {name}</b><br>")
                self.joint_status_text.append(f"  Current: {current:.4f} rad, Requested: {requested:.4f} rad, Change: {diff:.4f} rad")
                self.joint_status_text.append("")
        
        # Get time from start
        time_sec = int(self.time_from_start_input.value())
        time_nanosec = int((self.time_from_start_input.value() - time_sec) * 1e9)
        
        # Build the ros2 topic pub command
        cmd = f"ros2 topic pub --once {planned_trajectory_topic} trajectory_msgs/msg/JointTrajectory \"{{header: {{stamp: {{sec: 0, nanosec: 0}}, frame_id: ''}}, joint_names: ['arm_shoulder_pan_joint', 'arm_shoulder_lift_joint', 'arm_elbow_joint', 'arm_wrist_1_joint', 'arm_wrist_2_joint', 'arm_wrist_3_joint'], points: [{{positions: [{positions_str}], velocities: [], accelerations: [], effort: [], time_from_start: {{sec: {time_sec}, nanosec: {time_nanosec}}}}}]}}\""
        
        process = QProcess(self)
        process.setProcessChannelMode(QProcess.MergedChannels)
        process.readyReadStandardOutput.connect(lambda: self._handle_joint_publish_output(process))
        
        # Display command in bold green
        self._append_to_text_widget(self.joint_status_text, "<b style='color: #57ab5a;'>📤 Publishing joint trajectory...</b><br>", add_newline=True)
        
        # Show the positions being published
        self.joint_status_text.append(f"Positions: {positions_str}")
        self.joint_status_text.append(f"Time from start: {self.time_from_start_input.value():.1f} sec")
        
        process.finished.connect(lambda: self.on_joint_publish_finished(process))
        process.start('bash', ['-c', cmd])
 

    def _handle_joint_publish_output(self, process):
        """Handle output from joint trajectory publish command"""
        output = process.readAllStandardOutput().data().decode('utf-8')
        if output:
            # Display output line by line to preserve formatting
            lines = output.split('\n')
            for line in lines:
                html_line = self._ansi_to_html(line)
                self._append_to_text_widget(self.joint_status_text, html_line)

    def on_joint_publish_finished(self, process):
        """Handle completion of joint trajectory publish command"""
        exit_code = process.exitCode()
        
        if exit_code == 0:
            self.joint_status_text.append("")
            self.joint_status_text.insertHtml("<span style='color: #57ab5a;'>✓ Joint trajectory published successfully</span><br>")
            self.joint_status_text.append("")
            
            # Wait a moment for the robot to move, then re-read positions silently
            from PyQt5.QtCore import QTimer
            QTimer.singleShot(1500, lambda: self.read_joint_positions(silent=True))
        else:
            self.joint_status_text.append("")
            self.joint_status_text.insertHtml(f"<span style='color: #f47067;'>✗ Publish failed with exit code {exit_code}</span><br>")
            self.joint_status_text.append("")
            


    def send_goal(self):
        if not rclpy.ok():
            self._log_append(self.status_text, "<span style='color: #c69026;'>⚠ ROS context invalid - cannot publish</span>")
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
            self._log_append(self.status_text, f"Sent goal: pos({pose.position.x:.2f}, {pose.position.y:.2f}, {pose.position.z:.2f}) orn({pose.orientation.x:.2f}, {pose.orientation.y:.2f}, {pose.orientation.z:.2f}, {pose.orientation.w:.2f})")
        except Exception as e:
            self._log_append(self.status_text, f"<span style='color: #f47067;'>❌ Failed to send goal: {e}</span>")
 
    def send_position_command(self):
        """Send selected position using dynamic send_position service endpoint (Arm Control tab)."""
        position_name = self.position_dropdown.currentText()
        self._send_position_service_call(
            position_name=position_name,
            status_text=self.status_text,
            output_handler=self.handle_output,
            ui_context='arm',
        )

    def send_full_control_position_command(self):
        """Send selected position using dynamic send_position service endpoint (Full Control tab)."""
        position_name = self.full_control_position_dropdown.currentText()
        self._send_position_service_call(
            position_name=position_name,
            status_text=self.full_control_status_text,
            output_handler=self.handle_full_control_output,
            ui_context='full',
        )

    def reset_planner_arm(self):
        """Reset the planner from Arm Control tab"""
        self._reset_planner_generic('reset_planner_arm', self.status_text, self.handle_output)

    def reset_planner(self):
        """Reset the planner from Full Control tab"""
        self._reset_planner_generic('reset_planner_full', self.full_control_status_text, self.handle_full_control_output)

    def reset_planner_joint(self):
        """Reset the planner from Joint Control tab"""
        self._reset_planner_generic('reset_planner_joint', self.joint_status_text, self.handle_joint_output)
    
    def _reset_planner_generic(self, process_key, status_text_widget, output_handler):
        """Generic method to reset the planner by publishing to /planner/reset topic"""
        # Clean up existing process if any
        if process_key in self.process_map:
            existing_process = self.process_map[process_key]
            if existing_process.state() == QProcess.Running:
                existing_process.kill()
                existing_process.waitForFinished(1000)
            del self.process_map[process_key]
        
        # Create and start the process
        process = QProcess()
        command = 'ros2'
        args = ['topic', 'pub', '--once', '/planner/reset', 'std_msgs/msg/Bool', '{data: true}']
        
        # Display command in bold green
        cmd_str = command + ' ' + ' '.join(args)
        self._log_append(status_text_widget, f"<b style='color: #57ab5a;'>▶ {cmd_str}</b>")
        
        # Connect output handler
        process.readyReadStandardOutput.connect(lambda: output_handler(process))
        process.readyReadStandardError.connect(lambda: output_handler(process))
        process.finished.connect(lambda: self._cleanup_reset_planner(process_key, status_text_widget))
        
        # Start the process
        process.start(command, args)
        self.process_map[process_key] = process
    
    def _cleanup_reset_planner(self, process_key, status_text_widget):
        """Clean up reset planner process"""
        if process_key in self.process_map:
            process = self.process_map[process_key]
            exit_code = process.exitCode()
            if exit_code == 0:
                self._append_to_text_widget(
                    status_text_widget,
                    f"<span style='color: #3fb950;'>[Reset Planner]</span> Successfully reset planner."
                )
            else:
                self._append_to_text_widget(
                    status_text_widget,
                    f"<span style='color: #f85149;'>[Reset Planner]</span> Command failed with exit code {exit_code}."
                )
            del self.process_map[process_key]

    def _is_hybrid_sim_enabled(self, context='full'):
        """Return True when the requested tab is using hybrid / URSim mode."""
        if context == 'arm':
            if not hasattr(self, "arm_hybrid_sim_combo"):
                return False
            return self.arm_hybrid_sim_combo.currentText() == 'true'

        if not hasattr(self, "full_control_sim_mode_combo"):
            return False
        return self._sync_full_control_hybrid_sim_state() == 'true'

    def _get_joint_states_topic_for_ui(self, context='arm'):
        """Pick the joint_states topic for slider updates/readback based on context and simulation settings."""
        if context == 'arm':
            planner_backend = self._get_planner_backend(context='arm')
            if planner_backend == 'moveit':
                return '/arm/joint_states'
            return '/joint_states'
        # Full Control tab uses namespace when both simulation and hybrid_sim are true
        if (
            hasattr(self, 'full_control_sim_mode_combo')
            and self.full_control_sim_mode_combo.currentText() == 'true'
            and self._sync_full_control_hybrid_sim_state() == 'true'
        ):
            return '/arm/joint_states'
        return '/joint_states'

    def _get_planned_trajectory_topic_for_ui(self):
        """Pick the planned_trajectory topic for Joint Control publishing."""
        if self._full_control_uses_arm_namespace():
            return '/arm/planned_trajectory'
        return '/planned_trajectory'

    def _get_robot_ip_for_launch(self, context='arm'):
        """Pick robot_ip based on the tab's hybrid/URSim selector."""
        if self._is_hybrid_sim_enabled(context=context):
            return '192.168.56.101'
        return '192.168.1.102'

    def _get_planner_backend(self, context='arm'):
        """Pick planner_backend from the requested tab."""
        if context == 'full':
            if hasattr(self, 'full_control_planner_backend_combo'):
                return self.full_control_planner_backend_combo.currentText()
            return 'legacy'

        if hasattr(self, 'arm_planner_backend_combo'):
            return self.arm_planner_backend_combo.currentText()
        return 'legacy'

    def _get_moveit_planning_pipeline(self, context='arm'):
        """Pick the MoveIt planning pipeline from the requested tab."""
        if context == 'full':
            if hasattr(self, 'full_control_moveit_pipeline_combo'):
                return self.full_control_moveit_pipeline_combo.currentText()
        elif hasattr(self, 'arm_moveit_pipeline_combo'):
            return self.arm_moveit_pipeline_combo.currentText()
        return 'pilz_industrial_motion_planner'

    def _get_moveit_planner_id(self, context='arm'):
        """Pick the MoveIt planner id from the requested tab."""
        if context == 'full':
            if hasattr(self, 'full_control_moveit_planner_id_combo'):
                return self.full_control_moveit_planner_id_combo.currentText()
        elif hasattr(self, 'arm_moveit_planner_id_combo'):
            return self.arm_moveit_planner_id_combo.currentText()
        return 'PTP'

    def _get_moveit_launch_args(self, context='arm'):
        """Build launch arguments for the selected MoveIt pipeline and planner id."""
        if self._get_planner_backend(context=context) != 'moveit':
            return []

        planning_pipeline = self._get_moveit_planning_pipeline(context=context)
        moveit_args = [f'moveit_planning_pipeline:={planning_pipeline}']

        if planning_pipeline == 'pilz_industrial_motion_planner':
            pose_planner_id = self._get_moveit_planner_id(context=context)
            moveit_args.append(f'moveit_pose_planner_id:={pose_planner_id}')
            moveit_args.append('moveit_joint_planner_id:=PTP')
        else:
            moveit_args.append('moveit_pose_planner_id:=RRTConnectkConfigDefault')
            moveit_args.append('moveit_joint_planner_id:=RRTConnectkConfigDefault')

        return moveit_args

    def _get_namespace_for_arm_launch(self, planner_backend):
        """Arm launches now stay in the root namespace for both backends."""
        return ''

    def _full_control_uses_arm_namespace(self):
        """Only the Full Control hybrid launch keeps the arm stack under /arm."""
        return (
            hasattr(self, 'full_control_sim_mode_combo') and
            self.full_control_sim_mode_combo.currentText() == 'true' and
            self._sync_full_control_hybrid_sim_state() == 'true'
        )

    def _get_robot_ip_for_arm_launch(self):
        """Backward-compatible arm launch helper."""
        return self._get_robot_ip_for_launch(context='arm')

    def _get_emergency_stop_topic_for_ui(self, context='arm'):
        """Pick the emergency_stop topic based on context and simulation settings."""
        if context == 'arm':
            planner_backend = self._get_planner_backend(context='arm')
            if planner_backend == 'moveit':
                return '/arm/emergency_stop'
            return '/emergency_stop'
        # Full Control tab uses namespace when both simulation and hybrid_sim are true
        if (
            hasattr(self, 'full_control_sim_mode_combo')
            and self.full_control_sim_mode_combo.currentText() == 'true'
            and self._sync_full_control_hybrid_sim_state() == 'true'
        ):
            return '/arm/emergency_stop'
        return '/emergency_stop'

    def _get_status_text_for_context(self, context='arm'):
        """Get the appropriate status text widget based on context."""
        if context == 'full':
            return self.full_control_status_text
        return self.status_text

    def _get_emergency_stop_button_for_context(self, context='arm'):
        """Get the appropriate emergency stop button based on context."""
        if context == 'full' and hasattr(self, 'btn_full_control_emergency_stop'):
            return self.btn_full_control_emergency_stop
        return self.btn_emergency_stop

    def _get_send_position_service_name(self, context='arm'):
        """Pick send_position service name based on the active tab and planner backend."""
        if context == 'arm':
            planner_backend = self._get_planner_backend(context='arm')
            namespace = self._get_namespace_for_arm_launch(planner_backend)
            if namespace:
                return f'/{namespace}/send_position'
            return '/send_position'
        if self._is_hybrid_sim_enabled(context=context):
            return '/arm/send_position'
        return '/send_position'

    def _send_position_service_call(self, position_name, status_text, output_handler, ui_context='arm'):
        """Execute ros2 service call for the selected send_position service and stream output."""
        payload = f"{{position_name: '{position_name}'}}"
        service_name = self._get_send_position_service_name(context=ui_context)
        ros2_args = [
            'service',
            'call',
            service_name,
            'arm_control/srv/SendPosition',
            payload,
        ]

        cmd_str = f'ros2 service call {service_name} arm_control/srv/SendPosition "{payload}"'
        self._log_append(status_text, f"<b style='color: #57ab5a;'>▶ {cmd_str}</b>")

        process = QProcess(self)
        process.setProcessChannelMode(QProcess.MergedChannels)
        process.readyReadStandardOutput.connect(lambda: output_handler(process))

        process_key = f"send_position_service_{time.time_ns()}"
        process.finished.connect(
            lambda: self._cleanup_send_position_service_call(process_key, position_name, status_text)
        )

        process.start('ros2', ros2_args)
        self.process_map[process_key] = process

    def _cleanup_send_position_service_call(self, process_key, position_name, status_text):
        """Clean up one-shot send position service call process."""
        if process_key not in self.process_map:
            return

        exit_code = self.process_map[process_key].exitCode()
        del self.process_map[process_key]

        if exit_code == 0:
            self._log_append(status_text, f"✓ Position '{position_name}' request completed")
        else:
            self._log_append(status_text,
                f"<span style='color: #c69026;'>⚠ Position '{position_name}' request finished with exit code {exit_code}</span>"
            )
 
    def emergency_stop(self, context='arm'):
        """Toggle emergency stop state (latched)."""
        self.set_emergency_stop_state(not self.emergency_stop_active, source="ui", context=context)
            
    def _on_emergency_stop_state(self, msg: Bool):
        """Sync UI with the latched /arm/emergency_stop state (even if published externally)."""
        self.set_emergency_stop_state(bool(msg.data), source="topic", context='arm')

    def set_emergency_stop_state(self, active: bool, source: str = "ui", context: str = "arm"):
        """
        Centralized state setter.

        Behavior (matches old behavior):
        - Only publishes to emergency_stop topic when source == "ui"
        - Only publishes when the state actually changes
        - Topic callbacks only update UI (no re-publish), preventing feedback loops
        - Topic namespace depends on context and simulation settings
        """
        # If no change, do nothing (prevents repeated publishes / UI churn)
        if active == getattr(self, "emergency_stop_active", False):
            return

        # Update internal state + UI
        self.emergency_stop_active = active
        self._update_emergency_stop_button_ui(context=context)
        # QApplication.processEvents()

        # If this came from the topic, do not publish or trigger side effects
        if source != "ui":
            return

        # Get the appropriate status text widget for this context
        status_text = self._get_status_text_for_context(context)

        if not rclpy.ok():
            self._log_append(status_text, "<span style='color: #c69026;'>⚠ ROS context invalid - cannot set emergency stop</span>")
            return

        # Determine the correct topic based on context and simulation settings
        emergency_stop_topic = self._get_emergency_stop_topic_for_ui(context)

        # Display the command being executed in green color
        action_str = "true" if active else "false"
        cmd_str = f'ros2 topic pub --once {emergency_stop_topic} std_msgs/msg/Bool "{{data: {action_str}}}"'
        self._log_append(status_text, f"<b style='color: #57ab5a;'>▶ {cmd_str}</b>")

        # Publish only on user-triggered change
        try:
            stop_msg = Bool()
            stop_msg.data = active

            # Get or create a persistent publisher for the determined topic
            publisher = self._emergency_stop_publishers.get(emergency_stop_topic)
            if publisher is None:
                qos_profile = QoSProfile(
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                    history=HistoryPolicy.KEEP_LAST,
                    depth=10,
                )
                publisher = self.node.create_publisher(Bool, emergency_stop_topic, qos_profile)
                self._emergency_stop_publishers[emergency_stop_topic] = publisher

            publisher.publish(stop_msg)

        except Exception as e:
            self._log_append(status_text, f"<span style='color: #f47067;'>❌ Failed to publish {emergency_stop_topic}: {e}</span>")
            return

        # User-triggered side-effects only
        try:
            if active:
                self._log_append(status_text, "<span style='color: #c69026; font-weight: bold;'>⚠ EMERGENCY STOP - Published stop signal</span>")

                if self.current_goal_handle is not None:
                    self.current_goal_handle.cancel_goal_async()
                    self._log_append(status_text, "<span style='color: #c69026;'>⚠ Canceling current trajectory goal...</span>")
                    self.current_goal_handle = None

                response = self._send_robot_command('stop', status_text=status_text)
                if response:
                    self._log_append(status_text, "<span style='color: #c69026; font-weight: bold;'>⚠ Robot protective stop triggered</span>")

            else:
                self._log_append(status_text, "<span style='color: #57ab5a; font-weight: bold;'>✓ EMERGENCY STOP RELEASED - Published release signal</span>")

                self._send_robot_command('close safety popup', status_text=status_text)
                response = self._send_robot_command('unlock protective stop', status_text=status_text)
                if response:
                    self._log_append(status_text, "<span style='color: #57ab5a; font-weight: bold;'>✓ Robot protective stop released (requested)</span>")

                    play_resp = self._send_robot_command('play', status_text=status_text)
                    if play_resp:
                        self._log_append(status_text, "<span style='color: #57ab5a; font-weight: bold;'>✓ Robot program started (play)</span>")

        except Exception as e:
            self._log_append(status_text, f"<span style='color: #f47067;'>❌ Error while applying emergency stop state: {e}</span>")


    def _update_emergency_stop_button_ui(self, context='arm'):
        """Update button label + color according to emergency stop state.
        Updates both buttons to keep them in sync."""
        if self.emergency_stop_active:
            text = "EMERGENCY STOP ACTIVE (Click to Release)"
            style = "background-color: #8b0000; color: white; font-weight: bold;"
            checked = True
        else:
            text = "EMERGENCY STOP (Click to Activate)"
            style = "background-color: red; color: white; font-weight: bold;"
            checked = False

        # Update Arm Control button
        if hasattr(self, "btn_emergency_stop"):
            self.btn_emergency_stop.setText(text)
            self.btn_emergency_stop.setStyleSheet(style)
            self.btn_emergency_stop.setChecked(checked)

        # Update Full Control button
        if hasattr(self, "btn_full_control_emergency_stop"):
            self.btn_full_control_emergency_stop.setText(text)
            self.btn_full_control_emergency_stop.setStyleSheet(style)
            self.btn_full_control_emergency_stop.setChecked(checked)
 
    def closeEvent(self, event):
        self.timer.stop()

        # Detach every QProcess from this window so Qt does NOT send SIGTERM
        # to the underlying OS processes when the window is destroyed.
        for process in list(self.process_map.values()):
            try:
                process.setParent(None)
            except Exception:
                pass
        self.process_map.clear()

        for proc in (self.fsm_launch_process, self.fsm_node_process):
            if proc is not None:
                try:
                    proc.setParent(None)
                except Exception:
                    pass
        self.fsm_launch_process = None
        self.fsm_node_process = None

        # Close robot socket if connected
        if self.robot_socket:
            try:
                self.robot_socket.close()
            except Exception:
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
            'align_ee_to_wall',
            'robot_state_publisher',
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

    def _kill_gazebo_processes(self):
        """Kill newer Gazebo (Ignition/gz) processes."""
        # List of Gazebo/Ignition process patterns to kill
        gz_patterns = [
            'gz sim',
            'ign gazebo',
            'ruby.*gz',
            'gzserver',
            'gz-sim',
        ]
        
        for pattern in gz_patterns:
            try:
                subprocess.run(['pkill', '-9', '-f', pattern], timeout=2, stderr=subprocess.DEVNULL)
            except:
                pass
    
    def populate_process_combo(self, ps_output):
        """Parse ps output and populate the process combobox"""
        self.current_process_list = []
        self.process_combo.clear()
        
        if not ps_output:
            self.process_combo.addItem("No processes found")
            return
        
        lines = ps_output.strip().split('\n')
        item_index = 0
        for line in lines:
            if not line.strip():
                continue
            
            # Parse ps aux output: USER PID %CPU %MEM VSZ RSS TTY STAT START TIME COMMAND
            parts = line.split(None, 10)  # Split into max 11 parts
            if len(parts) >= 11:
                pid = parts[1]
                command = parts[10]
                
                # Store process info and add to combobox
                process_info = {'pid': pid, 'command': command, 'full_line': line}
                self.current_process_list.append(process_info)
                
                # Display only PID in the combobox
                display_text = f"PID: {pid}"
                self.process_combo.addItem(display_text)
                
                # Set tooltip to show full command
                tooltip = f"PID: {pid}\nCommand: {command}"
                self.process_combo.setItemData(item_index, tooltip, Qt.ToolTipRole)
                item_index += 1
        
        if not self.current_process_list:
            self.process_combo.addItem("No processes found")
        else:
            self._log_append(self.full_control_status_text,
                             f"<span style='color: #57ab5a;'>Found {len(self.current_process_list)} ROS2 processes</span>")

    def kill_selected_process(self):
        """Kill the process selected in the combobox"""
        current_index = self.process_combo.currentIndex()

        if current_index < 0 or current_index >= len(self.current_process_list):
            self._log_append(self.full_control_status_text,
                             "<span style='color: #d73a49;'>No process selected or invalid selection</span>")
            return

        process_info = self.current_process_list[current_index]
        pid = process_info['pid']
        command = process_info['command']

        # Confirm and kill
        from PyQt5.QtWidgets import QMessageBox
        reply = QMessageBox.question(
            self,
            'Confirm Kill Process',
            f"Are you sure you want to kill process {pid}?\n\nCommand: {command[:100]}",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )

        if reply == QMessageBox.Yes:
            # Execute kill command
            kill_process = QProcess(self)
            kill_process.setProcessChannelMode(QProcess.MergedChannels)

            cmd_str = f'kill -9 {pid}'
            self._log_append(self.full_control_status_text,
                             f"<b style='color: #d73a49;'>▶ {cmd_str}</b>")

            kill_process.finished.connect(lambda: self._on_kill_process_finished(pid, kill_process))
            kill_process.start('kill', ['-9', pid])

    def _on_kill_process_finished(self, pid, process):
        """Handle kill process completion"""
        exit_code = process.exitCode()

        if exit_code == 0:
            self._log_append(self.full_control_status_text,
                             f"<span style='color: #57ab5a;'>✓ Successfully killed process {pid}</span>")
            # Refresh the process list after killing
            QTimer.singleShot(500, self.run_full_control_ps_ros)
        else:
            error_output = process.readAllStandardOutput().data().decode()
            self._log_append(self.full_control_status_text,
                             f"<span style='color: #d73a49;'>✗ Failed to kill process {pid}: {error_output}</span>")

    def kill_all_processes(self):
        """Kill all detected processes except UI itself and ros2 daemon"""
        if not self.current_process_list:
            self._log_append(self.full_control_status_text,
                             "<span style='color: #d73a49;'>No processes to kill</span>")
            return

        # Get our own PID to exclude
        ui_pid = os.getpid()

        # Filter processes to kill (exclude UI and ros2 daemon)
        processes_to_kill = []
        for process_info in self.current_process_list:
            pid = process_info['pid']
            command = process_info['command'].lower()

            # Skip our own process
            if int(pid) == ui_pid:
                continue

            # Skip ros2 daemon
            if 'ros2' in command and 'daemon' in command:
                continue

            # Skip if it's the UI.py script
            if 'ui.py' in command:
                continue

            processes_to_kill.append(process_info)

        if not processes_to_kill:
            self._log_append(self.full_control_status_text,
                             "<span style='color: #d73a49;'>No processes to kill (all are protected)</span>")
            return

        # Confirm kill all
        from PyQt5.QtWidgets import QMessageBox
        reply = QMessageBox.question(
            self,
            'Confirm Kill All Processes',
            f"Are you sure you want to kill {len(processes_to_kill)} process(es)?\n\n"
            f"This will kill all detected processes except the UI and ros2 daemon.",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )

        if reply == QMessageBox.Yes:
            killed_count = 0
            failed_count = 0

            self._log_append(self.full_control_status_text,
                             f"<b style='color: #d73a49;'>▶ Killing {len(processes_to_kill)} process(es)...</b>")

            for process_info in processes_to_kill:
                pid = process_info['pid']
                command = process_info['command']

                try:
                    # Use os.kill for synchronous killing
                    os.kill(int(pid), signal.SIGKILL)
                    self._log_append(self.full_control_status_text,
                                     f"<span style='color: #57ab5a;'>✓ Killed PID {pid}: {command[:60]}...</span>")
                    killed_count += 1
                except ProcessLookupError:
                    self._log_append(self.full_control_status_text,
                                     f"<span style='color: #e3b341;'>⚠ Process {pid} already terminated</span>")
                    failed_count += 1
                except PermissionError:
                    self._log_append(self.full_control_status_text,
                                     f"<span style='color: #d73a49;'>✗ Permission denied for PID {pid}</span>")
                    failed_count += 1
                except Exception as e:
                    self._log_append(self.full_control_status_text,
                                     f"<span style='color: #d73a49;'>✗ Failed to kill PID {pid}: {str(e)}</span>")
                    failed_count += 1

            self._log_append(self.full_control_status_text,
                             f"<b style='color: #57ab5a;'>Finished: {killed_count} killed, {failed_count} failed</b>")
            
            # Refresh the process list after killing
            QTimer.singleShot(500, self.run_full_control_ps_ros)
 
    # ───────────────────────── FSM TAB ─────────────────────────

    def _create_fsm_tab(self):
        """Create the FSM control tab."""
        fsm_tab = QWidget()
        fsm_tab_layout = QVBoxLayout(fsm_tab)

        # ── Controls row ──
        controls_row = QHBoxLayout()

        controls_row.addWidget(QLabel("Sim Mode:"))
        self.fsm_sim_combo = QComboBox()
        self.fsm_sim_combo.addItems(["true", "false"])
        controls_row.addWidget(self.fsm_sim_combo)

        controls_row.addSpacing(16)
        controls_row.addWidget(QLabel("Planner Backend:"))
        self.fsm_planner_combo = QComboBox()
        self.fsm_planner_combo.addItems(["legacy", "moveit"])
        controls_row.addWidget(self.fsm_planner_combo)

        controls_row.addSpacing(16)
        controls_row.addWidget(QLabel("Initial State:"))
        self.fsm_state_combo = QComboBox()
        self.fsm_state_combo.addItems([
            "ScanWall", "CreateMap", "ExhaustiveScan",
            "Armfolding", "ArmUnfolding", "NavigateToPose", "BasePlacement",
        ])
        controls_row.addWidget(self.fsm_state_combo)

        controls_row.addSpacing(24)
        self.btn_fsm_start = QPushButton("Start FSM")
        self.btn_fsm_start.clicked.connect(self._toggle_fsm)
        controls_row.addWidget(self.btn_fsm_start)

        controls_row.addStretch()
        fsm_tab_layout.addLayout(controls_row)

        # ── Status header ──
        fsm_status_header = QHBoxLayout()
        fsm_status_header.addWidget(QLabel("FSM Output"))
        fsm_status_header.addStretch()

        fsm_status_header.addWidget(QLabel("Filter:"))
        self.fsm_filter_input = QLineEdit()
        self.fsm_filter_input.setPlaceholderText("Filter output lines...")
        self.fsm_filter_input.setMaximumWidth(200)
        self.fsm_filter_input.textChanged.connect(self._fsm_apply_filter)
        fsm_status_header.addWidget(self.fsm_filter_input)

        self.btn_restore_fsm_status = QPushButton("Restore")
        self.btn_restore_fsm_status.clicked.connect(self.restore_fsm_status)
        self.btn_restore_fsm_status.setMaximumWidth(80)
        self.btn_restore_fsm_status.setVisible(False)
        fsm_status_header.addWidget(self.btn_restore_fsm_status)

        btn_clear_fsm_status = QPushButton("Clear")
        btn_clear_fsm_status.clicked.connect(self.clear_fsm_status)
        btn_clear_fsm_status.setMaximumWidth(80)
        fsm_status_header.addWidget(btn_clear_fsm_status)

        fsm_tab_layout.addLayout(fsm_status_header)

        # ── Status output (full width, wrapped) ──
        self.fsm_status_text = QTextEdit()
        self.fsm_status_text.setReadOnly(True)
        self.fsm_status_text.setAcceptRichText(True)
        self.fsm_status_text.setLineWrapMode(QTextEdit.WidgetWidth)
        self.fsm_status_text.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.fsm_status_text.setStyleSheet(
            "background-color: #22272e; color: #adbac7; border: 1px solid #444c56; "
            "font-family: 'Courier New', monospace; white-space: pre-wrap;"
        )
        font_metrics = QFontMetrics(self.fsm_status_text.font())
        self.fsm_status_text.setTabStopDistance(font_metrics.horizontalAdvance(' ') * 8)
        fsm_tab_layout.addWidget(self.fsm_status_text, 1)

        # ── Stdin input row ──
        stdin_row = QHBoxLayout()
        stdin_row.addWidget(QLabel("Send input:"))
        self.fsm_stdin_input = QLineEdit()
        self.fsm_stdin_input.setPlaceholderText("Type input for the running FSM process and press Enter...")
        self.fsm_stdin_input.setEnabled(False)
        self.fsm_stdin_input.returnPressed.connect(self._send_fsm_input)
        stdin_row.addWidget(self.fsm_stdin_input)
        self.btn_fsm_send_input = QPushButton("Send")
        self.btn_fsm_send_input.setMaximumWidth(70)
        self.btn_fsm_send_input.setEnabled(False)
        self.btn_fsm_send_input.clicked.connect(self._send_fsm_input)
        stdin_row.addWidget(self.btn_fsm_send_input)
        fsm_tab_layout.addLayout(stdin_row)

        return fsm_tab

    def _toggle_fsm(self):
        """Start or stop the FSM launch + node pair."""
        if self.fsm_launch_process is not None or self.fsm_node_process is not None:
            self._stop_fsm()
        else:
            self._start_fsm()

    def _start_fsm(self):
        sim = self.fsm_sim_combo.currentText()
        planner = self.fsm_planner_combo.currentText()
        state = self.fsm_state_combo.currentText()

        self.btn_fsm_start.setText("Stop FSM")
        self.btn_fsm_start.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;")
        self._fsm_set_input_enabled(True)

        # ── Step 1: launch file ──
        launch_args = ['launch', 'task_planner_fsm', 'task_planner.launch.py']
        self._fsm_append_log(
            f"<b style='color: #57ab5a;'>▶ ros2 {' '.join(launch_args)}</b>",
            f"▶ ros2 {' '.join(launch_args)}",
        )
        proc_launch = QProcess(self)
        proc_launch.setProcessChannelMode(QProcess.MergedChannels)
        proc_launch.readyReadStandardOutput.connect(
            lambda p=proc_launch: self._handle_fsm_output(p)
        )
        proc_launch.finished.connect(
            lambda code, status, p=proc_launch: self._on_fsm_launch_finished(p, code)
        )
        proc_launch.start('ros2', launch_args)
        self.fsm_launch_process = proc_launch

        # ── Step 2: fsm_node (delayed so launch has time to spin up) ──
        node_args = [
            'run', 'task_planner_fsm', 'fsm_node',
            '--sim', sim,
            '--planner-backend', planner,
            '--initial-state', state,
        ]
        QTimer.singleShot(3000, lambda: self._start_fsm_node(node_args))

    def _start_fsm_node(self, node_args):
        """Start the fsm_node process (called after launch delay)."""
        if self.fsm_launch_process is None:
            # Launch was already stopped before the timer fired
            return
        self._fsm_append_log(
            f"<b style='color: #57ab5a;'>▶ ros2 {' '.join(node_args)}</b>",
            f"▶ ros2 {' '.join(node_args)}",
        )
        proc_node = QProcess(self)
        proc_node.setProcessChannelMode(QProcess.MergedChannels)
        proc_node.readyReadStandardOutput.connect(
            lambda p=proc_node: self._handle_fsm_output(p)
        )
        proc_node.finished.connect(
            lambda code, status, p=proc_node: self._on_fsm_node_finished(p, code)
        )
        proc_node.start('ros2', node_args)
        self.fsm_node_process = proc_node

    def _stop_fsm(self):
        """Stop both FSM processes and their entire spawned process trees."""

        # ── Phase 1: collect all descendant PIDs BEFORE sending any signals.
        # Once a parent is killed its children are reparented to init (PID 1),
        # so pgrep -P <dead_parent> returns nothing. We must snapshot the tree now.
        def _collect_descendants(parent_pid):
            pids = []
            if not parent_pid:
                return pids
            try:
                r = subprocess.run(
                    ['pgrep', '-P', str(parent_pid)],
                    capture_output=True, text=True, timeout=1,
                )
                for s in r.stdout.strip().split('\n'):
                    if s.strip():
                        child = int(s)
                        pids.extend(_collect_descendants(child))
                        pids.append(child)
            except Exception:
                pass
            return pids

        procs = [p for p in (self.fsm_node_process, self.fsm_launch_process) if p is not None]
        proc_pids = []
        all_descendants = []
        for proc in procs:
            try:
                proc.finished.disconnect()
            except Exception:
                pass
            pid = proc.processId()
            proc_pids.append(pid)
            if pid:
                all_descendants.extend(_collect_descendants(pid))

        # ── Phase 2: graceful SIGINT to parent processes.
        for pid in proc_pids:
            if pid:
                try:
                    os.kill(pid, signal.SIGINT)
                except Exception:
                    pass
        for proc in procs:
            proc.waitForFinished(3000)

        # ── Phase 3: SIGKILL every collected descendant.
        for dpid in all_descendants:
            try:
                os.kill(dpid, signal.SIGKILL)
            except Exception:
                pass

        # ── Phase 4: force-kill the parent QProcesses if still alive.
        for proc in procs:
            if proc.state() == QProcess.Running:
                proc.terminate()
                proc.waitForFinished(1000)
            if proc.state() == QProcess.Running:
                proc.kill()
                proc.waitForFinished(1000)
            proc.deleteLater()

        # ── Phase 5: wipe out Gazebo and any remaining stragglers.
        self._kill_gazebo_processes()

        self.fsm_node_process = None
        self.fsm_launch_process = None
        self.btn_fsm_start.setText("Start FSM")
        self.btn_fsm_start.setStyleSheet("")
        self._fsm_set_input_enabled(False)
        self._fsm_append_log(
            "<span style='color: #e3b341;'>⏹ FSM stopped</span>",
            "⏹ FSM stopped",
        )

    def _handle_fsm_output(self, process):
        """Stream output from a FSM process into the status pane."""
        output = process.readAllStandardOutput().data().decode(errors='replace')
        if not output:
            return
        for line in output.split('\n'):
            if line.strip():
                self._fsm_append_log(self._ansi_to_html(line), line)

    def _on_fsm_launch_finished(self, process, exit_code):
        if process is not self.fsm_launch_process:
            return
        self.fsm_launch_process = None
        color = '#57ab5a' if exit_code == 0 else '#f47067'
        msg = f"Launch exited (code {exit_code})"
        self._fsm_append_log(f"<span style='color: {color};'>{msg}</span>", msg)
        process.deleteLater()
        if self.fsm_node_process is None:
            self.btn_fsm_start.setText("Start FSM")
            self.btn_fsm_start.setStyleSheet("")
            self._fsm_set_input_enabled(False)

    def _on_fsm_node_finished(self, process, exit_code):
        if process is not self.fsm_node_process:
            return
        self.fsm_node_process = None
        color = '#57ab5a' if exit_code == 0 else '#f47067'
        msg = f"fsm_node exited (code {exit_code})"
        self._fsm_append_log(f"<span style='color: {color};'>{msg}</span>", msg)
        process.deleteLater()
        if self.fsm_launch_process is None:
            self.btn_fsm_start.setText("Start FSM")
            self.btn_fsm_start.setStyleSheet("")
            self._fsm_set_input_enabled(False)

    def _fsm_set_input_enabled(self, enabled: bool):
        self.fsm_stdin_input.setEnabled(enabled)
        self.btn_fsm_send_input.setEnabled(enabled)
        if not enabled:
            self.fsm_stdin_input.clear()

    def _send_fsm_input(self):
        text = self.fsm_stdin_input.text()
        if not text:
            return
        # Prefer the node process; fall back to the launch process
        target = self.fsm_node_process or self.fsm_launch_process
        if target is None or target.state() != QProcess.Running:
            self._fsm_append_log(
                "<span style='color: #f47067;'>⚠ No running FSM process to send input to</span>",
                "⚠ No running FSM process to send input to",
            )
            return
        target.write((text + '\n').encode())
        self._fsm_append_log(
            f"<span style='color: #76e3ea;'>▷ {html.escape(text)}</span>",
            f"▷ {text}",
        )
        self.fsm_stdin_input.clear()

    def _fsm_append_log(self, html_content, plain_text, add_newline=True):
        """Store entry in the log buffer and display it if it passes the current filter."""
        self.fsm_log_entries.append((html_content, plain_text, add_newline))
        term = self.fsm_filter_input.text().strip().lower()
        if not term or term in plain_text.lower():
            self._append_to_text_widget(self.fsm_status_text, html_content, add_newline)

    def _fsm_apply_filter(self):
        """Rebuild the status pane showing only lines that match the filter text."""
        term = self.fsm_filter_input.text().strip().lower()
        scrollbar = self.fsm_status_text.verticalScrollBar()
        was_at_bottom = scrollbar.value() >= scrollbar.maximum() - 10
        self.fsm_status_text.clear()
        for h, plain, nl in self.fsm_log_entries:
            if not term or term in plain.lower():
                self._append_to_text_widget(self.fsm_status_text, h, nl)
        if was_at_bottom:
            scrollbar.setValue(scrollbar.maximum())

    def clear_fsm_status(self):
        self.fsm_log_backup = list(self.fsm_log_entries)
        self.fsm_log_entries.clear()
        self.fsm_status_text.clear()
        self.btn_restore_fsm_status.setVisible(True)

    def restore_fsm_status(self):
        if self.fsm_log_backup:
            self.fsm_log_entries = self.fsm_log_backup + self.fsm_log_entries
            self.fsm_log_backup = []
            self._fsm_apply_filter()
            self.btn_restore_fsm_status.setVisible(False)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    window = RobotControlUI()
    window.show()
    sys.exit(app.exec_())
