#!/usr/bin/env python3
import sys
import os
import subprocess
import signal
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
        self.goal_publisher = self.node.create_publisher(Pose, '/goal_pose', 10)
        self.emergency_stop_publisher = self.node.create_publisher(Bool, '/emergency_stop', 10)
        self.distance_sensor_publisher = self.node.create_publisher(Float32MultiArray, '/distance_sensors', 10)
        
        # Track processes and their associated buttons
        self.process_map = {}
        self.button_map = {}
        
        # Action client for canceling trajectory goals
        self.action_client = ActionClient(
            self.node, 
            FollowJointTrajectory, 
            '/scaled_joint_trajectory_controller/follow_joint_trajectory'
        )
        self.current_goal_handle = None
        
        self.setWindowTitle("Robot Control Panel")
        self.setGeometry(100, 100, 800, 600)
        
        # Central widget
        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        
        # Launch buttons - arranged horizontally
        layout.addWidget(QLabel("Launch Controls:"))
        
        launch_row1 = QHBoxLayout()
        self.btn_ur_control = QPushButton("Launch UR Control")
        self.btn_ur_control.clicked.connect(self.toggle_ur_control)
        launch_row1.addWidget(self.btn_ur_control)
        
        self.btn_trajectory_launch = QPushButton("Launch Scaled Joint Trajectory")
        self.btn_trajectory_launch.clicked.connect(self.toggle_trajectory)
        launch_row1.addWidget(self.btn_trajectory_launch)
        layout.addLayout(launch_row1)
        
        launch_row2 = QHBoxLayout()
        self.btn_planner = QPushButton("Launch Planner Node")
        self.btn_planner.clicked.connect(self.toggle_planner)
        launch_row2.addWidget(self.btn_planner)
        
        self.btn_end_effector = QPushButton("Launch End Effector Pose Node")
        self.btn_end_effector.clicked.connect(self.toggle_end_effector_pose)
        launch_row2.addWidget(self.btn_end_effector)
        layout.addLayout(launch_row2)
        
        launch_row3 = QHBoxLayout()
        self.btn_sensors_orientation = QPushButton("Launch Sensors Orientation Node")
        self.btn_sensors_orientation.clicked.connect(self.toggle_sensors_orientation)
        launch_row3.addWidget(self.btn_sensors_orientation)
        
        self.btn_rqt_controller = QPushButton("Launch RQT Joint Controller")
        self.btn_rqt_controller.clicked.connect(self.toggle_rqt_controller)
        launch_row3.addWidget(self.btn_rqt_controller)
        layout.addLayout(launch_row3)
        
        # Position inputs
        layout.addWidget(QLabel("Goal Position:"))
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
        layout.addLayout(pos_layout)
        
        # Orientation inputs
        layout.addWidget(QLabel("Goal Orientation (Quaternion):"))
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
        layout.addLayout(orn_layout)
        
        # Distance Sensor Simulation
        layout.addWidget(QLabel("Distance Sensors (Simulation):"))
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
        layout.addLayout(sensor_layout)
        
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
        layout.addLayout(tof_layout)
        
        # Publish sensors button
        btn_publish_sensors = QPushButton("Publish Sensor Data")
        btn_publish_sensors.clicked.connect(self.publish_sensor_data)
        layout.addWidget(btn_publish_sensors)
        
        # Send goal and emergency stop buttons
        goal_layout = QHBoxLayout()
        btn_send_goal = QPushButton("Send Goal Pose")
        btn_send_goal.clicked.connect(self.send_goal)
        goal_layout.addWidget(btn_send_goal)
        
        btn_emergency_stop = QPushButton("EMERGENCY STOP / Cancel Goals")
        btn_emergency_stop.setStyleSheet("background-color: red; color: white; font-weight: bold;")
        btn_emergency_stop.clicked.connect(self.emergency_stop)
        goal_layout.addWidget(btn_emergency_stop)
        layout.addLayout(goal_layout)
        
        # Status display
        self.status_text = QTextEdit()
        self.status_text.setReadOnly(True)
        layout.addWidget(QLabel("Status:"))
        layout.addWidget(self.status_text)
        
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
        
    def toggle_ur_control(self):
        self._toggle_process('ur_control', self.btn_ur_control, 'UR Control',
                            'ros2', ['launch', 'arm_control', 'ur_control.launch.py',
                                    'ur_type:=ur10e', 'robot_ip:=192.168.56.102',
                                    'use_fake_hardware:=true', 'launch_rviz:=true'])
    
    def toggle_trajectory(self):
        self._toggle_process('trajectory', self.btn_trajectory_launch, 'Scaled Joint Trajectory',
                            'ros2', ['launch', 'arm_control', 'test_scaled_joint_trajectory_planned.launch.py'])
        
    def toggle_planner(self):
        self._toggle_process('planner', self.btn_planner, 'Planner Node',
                            'ros2', ['run', 'arm_control', 'planner_node'])
    
    def toggle_end_effector_pose(self):
        self._toggle_process('end_effector', self.btn_end_effector, 'End Effector Pose Node',
                            'ros2', ['run', 'arm_control', 'end_effector_pose_node'])
    
    def toggle_sensors_orientation(self):
        self._toggle_process('sensors_orientation', self.btn_sensors_orientation, 'Sensors Orientation Node',
                            'ros2', ['run', 'arm_control', 'sensors_orientation'])
    
    def toggle_rqt_controller(self):
        self._toggle_process('rqt_controller', self.btn_rqt_controller, 'RQT Joint Controller',
                            'ros2', ['run', 'rqt_joint_trajectory_controller', 'rqt_joint_trajectory_controller', '--force-discover'])
    
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
            
            # For ur_control, kill rviz2 before terminating the launch process
            if process_key == 'ur_control':
                try:
                    subprocess.run(['pkill', '-9', 'rviz2'], timeout=2, stderr=subprocess.DEVNULL)
                except:
                    pass
            
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
            process.start(program, args)
            self.process_map[process_key] = process
            button.setText(f"Stop {name}")
            button.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;")
            self.status_text.append(f"▶ Launched {name}")
    
    def _on_process_finished(self, process_key, button, name):
        """Handle when a process finishes unexpectedly"""
        if process_key in self.process_map:
            del self.process_map[process_key]
            button.setText(f"Launch {name}")
            button.setStyleSheet("")
            self.status_text.append(f"⚠ {name} exited")
    
    def handle_output(self, process):
        output = process.readAllStandardOutput().data().decode()
        if output.strip():
            self.status_text.append(output.strip())
        
    def send_goal(self):
        if not rclpy.ok():
            self.status_text.append("⚠ ROS context invalid - cannot publish")
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
            self.status_text.append(f"⚠ Failed to send goal: {e}")
    
    def publish_sensor_data(self):
        """Publish simulated distance sensor data"""
        if not rclpy.ok():
            self.status_text.append("⚠ ROS context invalid - cannot publish")
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
            self.status_text.append(f"⚠ Failed to publish sensors: {e}")
    
    def emergency_stop(self):
        """Trigger emergency stop and cancel current trajectory goal"""
        if not rclpy.ok():
            self.status_text.append("⚠ ROS context invalid - cannot send emergency stop")
            return
        try:
            # Publish emergency stop signal
            stop_msg = Bool()
            stop_msg.data = True
            self.emergency_stop_publisher.publish(stop_msg)
            self.status_text.append("⚠️ EMERGENCY STOP - Published stop signal")
            
            # Cancel current goal if one exists
            if self.current_goal_handle is not None:
                cancel_future = self.current_goal_handle.cancel_goal_async()
                self.status_text.append("⚠️ Canceling current trajectory goal...")
                self.current_goal_handle = None
        except Exception as e:
            self.status_text.append(f"Error during emergency stop: {e}")
        
    def closeEvent(self, event):
        self.timer.stop()
        # Terminate all launched processes
        for process in self.process_map.values():
            if process.state() == QProcess.Running:
                process.terminate()
                process.waitForFinished(3000)
                if process.state() == QProcess.Running:
                    process.kill()
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = RobotControlUI()
    window.show()
    sys.exit(app.exec_())