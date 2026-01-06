#!/usr/bin/env python3
import sys
import os
import subprocess
import signal
import socket
import re
import rclpy
from rclpy.action import ActionClient
from PyQt5.QtWidgets import *
from PyQt5.QtCore import QTimer, QProcess
from PyQt5.QtGui import QIcon
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool, Float32MultiArray
from control_msgs.action import FollowJointTrajectory

class RobotControlUI(QMainWindow):
    def __init__(self):
        super().__init__()
        
        # Initialize ROS only if not already initialized
        if not rclpy.ok():
            rclpy.init()
        
        self.node = rclpy.create_node('robot_control_ui')
        self.goal_publisher = self.node.create_publisher(Pose, '/arm/goal_pose', 10)
        self.emergency_stop_publisher = self.node.create_publisher(Bool, '/arm/emergency_stop', 10)
        self.distance_sensor_publisher = self.node.create_publisher(Float32MultiArray, '/arm/distance_sensors', 10)
        
        # Track processes and their associated buttons
        self.process_map = {}
        self.button_map = {}
        
        # Store cleared status text for restore functionality
        self.cleared_status_backup = None
        self.base_cleared_status_backup = None
        
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
        self.btn_general_launch.setToolTip("ros2 launch arm_control general.launch.py use_fake_hardware:=true robot_ip:=192.168.1.102")
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
        
        btn_emergency_stop = QPushButton("EMERGENCY STOP / Cancel Goals")
        btn_emergency_stop.setStyleSheet("background-color: red; color: white; font-weight: bold;")
        btn_emergency_stop.clicked.connect(self.emergency_stop)
        btn_emergency_stop.setToolTip("Publish emergency stop to /arm/emergency_stop and cancel trajectory goals")
        goal_layout.addWidget(btn_emergency_stop)
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
            'stop',
            'close popup',
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
        
        # Distance Sensor Simulation
        sensors_layout.addWidget(QLabel("Distance Sensors (Simulation):"))
        
        # Arduino sensor node button
        self.btn_arduino_sensors = QPushButton("Launch Arduino Sensors")
        self.btn_arduino_sensors.clicked.connect(self.toggle_arduino_sensors)
        self.btn_arduino_sensors.setToolTip("ros2 run arm_control sensors_orientation_arduino --ros-args -r __ns:=/arm")
        sensors_layout.addWidget(self.btn_arduino_sensors)
        
        sensor_layout = QHBoxLayout()
        
        # Ultrasound sensors (in meters)
        self.ultra1_input = QDoubleSpinBox()
        self.ultra1_input.setRange(0, 2.55)
        self.ultra1_input.setValue(0.20)
        self.ultra1_input.setPrefix("U1(C): ")
        self.ultra1_input.setSuffix(" m")
        self.ultra1_input.setDecimals(3)
        self.ultra1_input.setSingleStep(0.01)
        
        self.ultra2_input = QDoubleSpinBox()
        self.ultra2_input.setRange(0, 2.55)
        self.ultra2_input.setValue(0.20)
        self.ultra2_input.setPrefix("U2(A): ")
        self.ultra2_input.setSuffix(" m")
        self.ultra2_input.setDecimals(3)
        self.ultra2_input.setSingleStep(0.01)
        
        self.ultra3_input = QDoubleSpinBox()
        self.ultra3_input.setRange(0, 2.55)
        self.ultra3_input.setValue(0.20)
        self.ultra3_input.setPrefix("U3(B): ")
        self.ultra3_input.setSuffix(" m")
        self.ultra3_input.setDecimals(3)
        self.ultra3_input.setSingleStep(0.01)
        
        sensor_layout.addWidget(self.ultra1_input)
        sensor_layout.addWidget(self.ultra2_input)
        sensor_layout.addWidget(self.ultra3_input)
        sensors_layout.addLayout(sensor_layout)
        
        # ToF sensors (in meters)
        tof_layout = QHBoxLayout()
        self.tof1_input = QDoubleSpinBox()
        self.tof1_input.setRange(0, 0.255)
        self.tof1_input.setValue(0.020)
        self.tof1_input.setPrefix("ToF1: ")
        self.tof1_input.setSuffix(" m")
        self.tof1_input.setDecimals(3)
        self.tof1_input.setSingleStep(0.001)
        
        self.tof2_input = QDoubleSpinBox()
        self.tof2_input.setRange(0, 0.255)
        self.tof2_input.setValue(0.020)
        self.tof2_input.setPrefix("ToF2: ")
        self.tof2_input.setSuffix(" m")
        self.tof2_input.setDecimals(3)
        self.tof2_input.setSingleStep(0.001)
        
        self.tof3_input = QDoubleSpinBox()
        self.tof3_input.setRange(0, 0.255)
        self.tof3_input.setValue(0.020)
        self.tof3_input.setPrefix("ToF3: ")
        self.tof3_input.setSuffix(" m")
        self.tof3_input.setDecimals(3)
        self.tof3_input.setSingleStep(0.001)
        
        tof_layout.addWidget(self.tof1_input)
        tof_layout.addWidget(self.tof2_input)
        tof_layout.addWidget(self.tof3_input)
        sensors_layout.addLayout(tof_layout)
        
        # Publish sensors button
        btn_publish_sensors = QPushButton("Publish Sensor Data")
        btn_publish_sensors.clicked.connect(self.publish_sensor_data)
        btn_publish_sensors.setToolTip("Publish Float32MultiArray to /arm/distance_sensors topic")
        sensors_layout.addWidget(btn_publish_sensors)
        
        sensors_layout.addStretch()
        boxes_layout.addWidget(sensors_box)
        
        # Status display for Arm Control tab
        self.status_text = QTextEdit()
        self.status_text.setReadOnly(True)
        self.status_text.setAcceptRichText(True)
        self.status_text.setStyleSheet("background-color: #22272e; color: #adbac7; border: 1px solid #444c56; font-family: 'Courier New', monospace; white-space: pre;")
        
        # Set tab stops to 8 characters (standard terminal width)
        from PyQt5.QtGui import QFontMetrics
        font_metrics = QFontMetrics(self.status_text.font())
        tab_width = font_metrics.horizontalAdvance(' ') * 8
        self.status_text.setTabStopDistance(tab_width)
        
        status_header = QHBoxLayout()
        status_header.addWidget(QLabel("Status:"))
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
        
        arm_tab_layout.addWidget(self.status_text)
        
        tabs.addTab(arm_tab, "Arm Control")
        
        # ===== BASE CONTROL TAB =====
        base_tab = QWidget()
        base_tab_layout = QVBoxLayout(base_tab)
        
        # Base Control Box
        base_control_box = QGroupBox("Base Navigation & Mapping")
        base_control_layout = QVBoxLayout()
        base_control_box.setLayout(base_control_layout)
        
        # Simulation parameter selector
        sim_param_layout = QHBoxLayout()
        sim_param_layout.addWidget(QLabel("Simulation Mode:"))
        self.sim_mode_combo = QComboBox()
        self.sim_mode_combo.addItems(['true', 'false'])
        sim_param_layout.addWidget(self.sim_mode_combo)
        sim_param_layout.addStretch()
        base_control_layout.addLayout(sim_param_layout)
        
        # Launch Mapping button
        self.btn_launch_mapping = QPushButton("Launch Mapping")
        self.btn_launch_mapping.clicked.connect(self.toggle_mapping)
        self.btn_launch_mapping.setToolTip("ros2 launch navi_wall mapping_3d.launch.py sim:=<mode> lidar:=sick")
        base_control_layout.addWidget(self.btn_launch_mapping)
        
        # Launch Localization button
        self.btn_launch_localization = QPushButton("Launch Localization")
        self.btn_launch_localization.clicked.connect(self.toggle_localization)
        self.btn_launch_localization.setToolTip("ros2 launch navi_wall move_robot.launch.py sim:=<mode>")
        base_control_layout.addWidget(self.btn_launch_localization)
        
        # Launch Nav2 button
        self.btn_launch_nav2 = QPushButton("Launch Nav2")
        self.btn_launch_nav2.clicked.connect(self.toggle_nav2)
        self.btn_launch_nav2.setToolTip("ros2 launch navi_wall navigation_launch.py use_sim_time:=<mode>")
        base_control_layout.addWidget(self.btn_launch_nav2)
        
        # Launch Exploration button
        self.btn_launch_exploration = QPushButton("Launch Exploration")
        self.btn_launch_exploration.clicked.connect(self.toggle_exploration)
        self.btn_launch_exploration.setToolTip("ros2 run navi_wall explore --ros-args --params-file <pkg>/config/explore_params.yaml")
        base_control_layout.addWidget(self.btn_launch_exploration)
        
        base_control_layout.addStretch()
        base_tab_layout.addWidget(base_control_box)
        
        # Status display for Base Control tab
        self.base_status_text = QTextEdit()
        self.base_status_text.setReadOnly(True)
        self.base_status_text.setAcceptRichText(True)
        self.base_status_text.setStyleSheet("background-color: #22272e; color: #adbac7; border: 1px solid #444c56; font-family: 'Courier New', monospace; white-space: pre;")
        
        # Set tab stops to 8 characters (standard terminal width)
        from PyQt5.QtGui import QFontMetrics
        font_metrics = QFontMetrics(self.base_status_text.font())
        tab_width = font_metrics.horizontalAdvance(' ') * 8
        self.base_status_text.setTabStopDistance(tab_width)
        
        base_status_header = QHBoxLayout()
        base_status_header.addWidget(QLabel("Status:"))
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
        
        base_tab_layout.addWidget(self.base_status_text)
        
        tabs.addTab(base_tab, "Base Control")
        
        # Timer for ROS spinning
        self.timer = QTimer()
        self.timer.timeout.connect(self._spin_ros)
        self.timer.start(100)  # 10 Hz
    
    def _spin_ros(self):
        """Safely spin ROS, checking context is valid first"""
        try:
            if rclpy.ok():
                rclpy.spin_once(self.node, timeout_sec=0)
        except Exception:
            pass  # Ignore errors if context is shutting down
    
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
        self._toggle_process('general_launch', self.btn_general_launch, 'General Launch',
                            'ros2', ['launch', 'arm_control', 'general.launch.py', 
                                    'use_fake_hardware:=true', 'robot_ip:=192.168.1.102'])
    
    def toggle_rqt_controller(self):
        self._toggle_process('rqt_controller', self.btn_rqt_controller, 'RQT Joint Controller',
                            'ros2', ['run', 'rqt_joint_trajectory_controller', 'rqt_joint_trajectory_controller', 
                                    '--force-discover', '--ros-args', '-r', '__ns:=/arm'])
    
    def toggle_arduino_sensors(self):
        self._toggle_process('arduino_sensors', self.btn_arduino_sensors, 'Arduino Sensors',
                            'ros2', ['run', 'arm_control', 'sensors_orientation_arduino', '--ros-args', '-r', '__ns:=/arm'])
    
    def toggle_mapping(self):
        sim_mode = self.sim_mode_combo.currentText()
        self._toggle_base_process('mapping', self.btn_launch_mapping, 'Mapping',
                                 'ros2', ['launch', 'navi_wall', 'mapping_3d.launch.py', 
                                         f'sim:={sim_mode}', 'lidar:=sick'])
    
    def toggle_localization(self):
        sim_mode = self.sim_mode_combo.currentText()
        self._toggle_base_process('localization', self.btn_launch_localization, 'Localization',
                                 'ros2', ['launch', 'navi_wall', 'move_robot.launch.py', 
                                         f'sim:={sim_mode}'])
    
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
            self.status_text.append(f"<b style='color: #57ab5a;'>▶ {cmd_str}</b>")
            
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
                pid = process.processId()
                if pid:
                    try:
                        subprocess.run(['pkill', '-9', '-P', str(pid)], timeout=2, stderr=subprocess.DEVNULL)
                    except:
                        pass
                self._cleanup_ros_children_of_pid(pid)
            
            process.terminate()
            process.waitForFinished(3000)
            if process.state() == QProcess.Running:
                process.kill()
            
            del self.process_map[process_key]
            button.setText(f"Launch {name}")
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
            self.base_status_text.append(f"<b style='color: #57ab5a;'>▶ {cmd_str}</b>")
            
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
    
    def handle_output(self, process):
        output = process.readAllStandardOutput().data().decode()
        if output.strip():
            # Color-code ROS log messages
            lines = output.strip().split('\n')
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
        if output.strip():
            # Color-code ROS log messages
            lines = output.strip().split('\n')
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
    
    def publish_sensor_data(self):
        """Publish simulated distance sensor data"""
        if not rclpy.ok():
            self.status_text.append("<span style='color: #c69026;'>⚠ ROS context invalid - cannot publish</span>")
            return
        try:
            msg = Float32MultiArray()
            # Order: [ultra1, ultra2, ultra3, tof1, tof2, tof3]
            msg.data = [
                self.ultra1_input.value(),
                self.ultra2_input.value(),
                self.ultra3_input.value(),
                self.tof1_input.value(),
                self.tof2_input.value(),
                self.tof3_input.value()
            ]
            self.distance_sensor_publisher.publish(msg)
            self.status_text.append(f"Published sensors: Ultra[{msg.data[0]:.3f}, {msg.data[1]:.3f}, {msg.data[2]:.3f}] ToF[{msg.data[3]:.3f}, {msg.data[4]:.3f}, {msg.data[5]:.3f}]")
        except Exception as e:
            self.status_text.append(f"<span style='color: #f47067;'>❌ Failed to publish sensors: {e}</span>")
    
    def send_position_command(self):
        """Send position by launching node with selected position as argument"""
        position_name = self.position_dropdown.currentText()
        
        # Build command
        args = ['run', 'arm_control', 'position_sender_node', 
                '--ros-args', '-r', '__ns:=/arm', '--', position_name]
        
        # Display command in bold green
        cmd_str = 'ros2 ' + ' '.join(args)
        self.status_text.append(f"<b style='color: #57ab5a;'>→ {cmd_str}</b>")
        
        # Launch the position_sender_node with the selected position
        process = QProcess(self)
        process.setProcessChannelMode(QProcess.MergedChannels)
        process.readyReadStandardOutput.connect(lambda: self.handle_output(process))
        
        # Store process temporarily to prevent garbage collection
        process_key = f'position_cmd_{position_name}'
        process.finished.connect(lambda: self._cleanup_position_sender(process_key, position_name))
        process.start('ros2', args)
        self.process_map[process_key] = process
    
    def _cleanup_position_sender(self, process_key, position_name):
        """Clean up finished position sender process"""
        if process_key in self.process_map:
            del self.process_map[process_key]
        self.status_text.append(f"✓ Position '{position_name}' command completed")
    
    def emergency_stop(self):
        """Trigger emergency stop and cancel current trajectory goal"""
        if not rclpy.ok():
            self.status_text.append("<span style='color: #c69026;'>⚠ ROS context invalid - cannot send emergency stop</span>")
            return
        try:
            # Publish emergency stop signal
            stop_msg = Bool()
            stop_msg.data = True
            self.emergency_stop_publisher.publish(stop_msg)
            self.status_text.append("<span style='color: #c69026; font-weight: bold;'>⚠️ EMERGENCY STOP - Published stop signal</span>")
            
            # Cancel current goal if one exists
            if self.current_goal_handle is not None:
                cancel_future = self.current_goal_handle.cancel_goal_async()
                self.status_text.append("<span style='color: #c69026;'>⚠️ Canceling current trajectory goal...</span>")
                self.current_goal_handle = None
        except Exception as e:
            self.status_text.append(f"<span style='color: #f47067;'>❌ Error during emergency stop: {e}</span>")
        
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