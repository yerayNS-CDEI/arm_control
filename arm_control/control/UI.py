#!/usr/bin/env python3
import sys
import os
import subprocess
import signal
import socket
import rclpy
from rclpy.action import ActionClient
from PyQt5.QtWidgets import *
from PyQt5.QtCore import QTimer, QProcess
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
        
        # Central widget
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        
        # Horizontal layout for three boxes side by side
        boxes_layout = QHBoxLayout()
        main_layout.addLayout(boxes_layout)
        
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
        
        # Position inputs
        control_layout.addWidget(QLabel("Goal Position:"))
        pos_layout = QHBoxLayout()
        self.x_input = QDoubleSpinBox()
        self.x_input.setRange(-2, 2)
        self.x_input.setValue(-0.4)
        self.x_input.setPrefix("X: ")
        
        self.y_input = QDoubleSpinBox()
        self.y_input.setRange(-2, 2)
        self.y_input.setValue(-0.7)
        self.y_input.setPrefix("Y: ")
        
        self.z_input = QDoubleSpinBox()
        self.z_input.setRange(0, 2)
        self.z_input.setValue(1.0)
        self.z_input.setPrefix("Z: ")
        
        pos_layout.addWidget(self.x_input)
        pos_layout.addWidget(self.y_input)
        pos_layout.addWidget(self.z_input)
        control_layout.addLayout(pos_layout)
        
        # Orientation inputs
        control_layout.addWidget(QLabel("Goal Orientation (Quaternion):"))
        orn_layout = QHBoxLayout()
        self.qx_input = QDoubleSpinBox()
        self.qx_input.setRange(-1, 1)
        self.qx_input.setValue(0.0)
        self.qx_input.setPrefix("QX: ")
        self.qx_input.setDecimals(4)
        self.qx_input.setSingleStep(0.01)
        
        self.qy_input = QDoubleSpinBox()
        self.qy_input.setRange(-1, 1)
        self.qy_input.setValue(-0.70710678)
        self.qy_input.setPrefix("QY: ")
        self.qy_input.setDecimals(4)
        self.qy_input.setSingleStep(0.01)
        
        self.qz_input = QDoubleSpinBox()
        self.qz_input.setRange(-1, 1)
        self.qz_input.setValue(0.0)
        self.qz_input.setPrefix("QZ: ")
        self.qz_input.setDecimals(4)
        self.qz_input.setSingleStep(0.01)
        
        self.qw_input = QDoubleSpinBox()
        self.qw_input.setRange(-1, 1)
        self.qw_input.setValue(0.70710678)
        self.qw_input.setPrefix("QW: ")
        self.qw_input.setDecimals(4)
        self.qw_input.setSingleStep(0.01)
        
        orn_layout.addWidget(self.qx_input)
        orn_layout.addWidget(self.qy_input)
        orn_layout.addWidget(self.qz_input)
        orn_layout.addWidget(self.qw_input)
        control_layout.addLayout(orn_layout)
        
        # Send goal and emergency stop buttons
        goal_layout = QHBoxLayout()
        btn_send_goal = QPushButton("Send Goal Pose")
        btn_send_goal.clicked.connect(self.send_goal)
        btn_send_goal.setToolTip("Publish Pose message to /arm/goal_pose topic")
        goal_layout.addWidget(btn_send_goal)
        
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
        
        # Status display (shared at bottom of main window)
        self.status_text = QTextEdit()
        self.status_text.setReadOnly(True)
        self.status_text.setAcceptRichText(True)
        self.status_text.setStyleSheet("background-color: #22272e; color: #adbac7; border: 1px solid #444c56;")
        
        status_header = QHBoxLayout()
        status_header.addWidget(QLabel("Status:"))
        status_header.addStretch()
        btn_clear_status = QPushButton("Clear")
        btn_clear_status.clicked.connect(self.status_text.clear)
        btn_clear_status.setMaximumWidth(80)
        status_header.addWidget(btn_clear_status)
        main_layout.addLayout(status_header)
        
        main_layout.addWidget(self.status_text)
        
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
            self.status_text.append(f"<b style='color: green;'>▶ {cmd_str}</b>")
            
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
            self.status_text.append(f"<span style='color: orange;'>⚠ {name} exited</span>")
    
    def handle_output(self, process):
        output = process.readAllStandardOutput().data().decode()
        if output.strip():
            # Color-code ROS log messages
            lines = output.strip().split('\n')
            for line in lines:
                # Skip expected shutdown messages (exit code -9 from our kill signals)
                if 'process has died' in line and 'exit code -9' in line:
                    continue
                    
                if '[WARN]' in line or '[WARNING]' in line:
                    self.status_text.append(f"<span style='color: orange;'>{line}</span>")
                elif '[ERROR]' in line or '[FATAL]' in line:
                    self.status_text.append(f"<span style='color: red;'>{line}</span>")
                else:
                    # INFO and other messages use default color
                    self.status_text.append(line)
    
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
                self.status_text.append(f"<span style='color: red;'>✗ Failed to connect to robot: {e}</span>")
                self.robot_socket = None
                return False
        return True
    
    def _send_robot_command(self, command):
        """Send command to robot dashboard and return response"""
        if not self._connect_robot_socket():
            return None
        
        try:
            self.robot_socket.send(str.encode(command + '\n'))
            self.status_text.append(f"<b style='color: green;'>→ SENT: {command}</b>")
            
            data = self.robot_socket.recv(1024)
            response = data.decode('utf-8').strip()
            self.status_text.append(f"← RECV: {response}")
            return response
        except Exception as e:
            self.status_text.append(f"<span style='color: red;'>✗ Command failed: {e}</span>")
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
        
    def send_goal(self):
        if not rclpy.ok():
            self.status_text.append("<span style='color: orange;'>⚠ ROS context invalid - cannot publish</span>")
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
            self.status_text.append(f"<span style='color: red;'>❌ Failed to send goal: {e}</span>")
    
    def publish_sensor_data(self):
        """Publish simulated distance sensor data"""
        if not rclpy.ok():
            self.status_text.append("<span style='color: orange;'>⚠ ROS context invalid - cannot publish</span>")
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
            self.status_text.append(f"<span style='color: red;'>❌ Failed to publish sensors: {e}</span>")
    
    def send_position_command(self):
        """Send position by launching node with selected position as argument"""
        position_name = self.position_dropdown.currentText()
        
        # Build command
        args = ['run', 'arm_control', 'position_sender_node', 
                '--ros-args', '-r', '__ns:=/arm', '--', position_name]
        
        # Display command in bold green
        cmd_str = 'ros2 ' + ' '.join(args)
        self.status_text.append(f"<b style='color: green;'>→ {cmd_str}</b>")
        
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
            self.status_text.append("<span style='color: orange;'>⚠ ROS context invalid - cannot send emergency stop</span>")
            return
        try:
            # Publish emergency stop signal
            stop_msg = Bool()
            stop_msg.data = True
            self.emergency_stop_publisher.publish(stop_msg)
            self.status_text.append("<span style='color: orange;'>⚠️ EMERGENCY STOP - Published stop signal</span>")
            
            # Cancel current goal if one exists
            if self.current_goal_handle is not None:
                cancel_future = self.current_goal_handle.cancel_goal_async()
                self.status_text.append("<span style='color: orange;'>⚠️ Canceling current trajectory goal...</span>")
                self.current_goal_handle = None
        except Exception as e:
            self.status_text.append(f"<span style='color: red;'>❌ Error during emergency stop: {e}</span>")
        
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