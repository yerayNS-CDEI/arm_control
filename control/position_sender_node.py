#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, ColorRGBA
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
import yaml
import os
from ament_index_python.packages import get_package_share_directory
from rclpy.utilities import remove_ros_args
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from arm_control.srv import SendPosition

import numpy as np
from scipy.spatial.transform import Rotation as R
from planner.planner_lib.closed_form_algorithm import closed_form_algorithm

class PositionSenderNode(Node):
    def __init__(self):
        super().__init__('position_sender_node')
        
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        # Publisher for goal pose
        self.publisher_ = self.create_publisher(PoseStamped, '/arm/goal_pose', 10)
        self.trajectory_pub = self.create_publisher(JointTrajectory, 'planned_trajectory', 10)
        self.marker_pub = self.create_publisher(Marker, 'goal_pose_marker', 10)
        
        # Subscriber for execution status
        # Subscribe to both namespaced and global topics to survive launch namespace drift.
        self.subscriptor_ = self.create_subscription(PoseStamped, 'end_effector_pose', self.end_effector_pose_callback, 10)
        self.subscriptor_global_ = self.create_subscription(PoseStamped, '/end_effector_pose', self.end_effector_pose_callback, 10)
        self.execution_status_sub = self.create_subscription(Bool,'execution_status',self.execution_status_callback,10)
        self.execution_status_sub_global = self.create_subscription(Bool,'/execution_status',self.execution_status_callback,10)
        self.emergency_sub = self.create_subscription(Bool, "emergency_stop", self.emergency_callback, qos)        
        self.create_subscription(JointState, "joint_states", self.joint_state_callback, 10)
        self.create_subscription(JointState, "/joint_states", self.joint_state_callback, 10)
        
        # Service for sending positions
        self.send_position_service = self.create_service(
            SendPosition,
            'send_position',
            self.send_position_service_callback
        )
        
        # Predefined positions (displaced 15cm along local +Z from original wrist_3 positions)
        self.positions = {
            'custom': {
                'joints': (0.0, -1.104, -2.034, 0.0, 1.57, 2.8),
                'pose': (0.56100, -0.17302, 0.85076, 0.406, 0.577, 0.408, 0.580)
            },
            'folded': {
                'joints': (3.1313, -1.6422, -2.4464, -2.7985, -0.6618, 0.2421),
                'pose': (-0.44101, 0.38792, 0.13674, -0.327, 0.753, 0.564, -0.087)
            },
            'unfolded': {
                'joints': (3.291, -2.169, -1.523, -2.790, -0.367, 0.167),
                'pose': (-0.95542, 0.28237, 0.25232, -0.193, 0.706, 0.661, -0.166)
            },
            'up': {
                'joints': (),
                'pose': (-0.31449, 0.36707, 0.62584, -0.067, 0.562, 0.821, -0.071)
            },
            'down': {
                'joints': (),
                'pose': (-0.30245, 0.31807, 0.35029, -0.105, 0.908, 0.404, -0.032)
            },
            'front': {
                'joints': (),
                'pose': (-0.31718, 0.38493, 0.53014, -0.083, 0.702, 0.705, -0.06)
            },
            'left': {   
                'joints': (),
                'pose': (-0.53364, 0.14888, 0.50193, -0.442, 0.549, 0.564, -0.430)
            },
            'right': {
                'joints': (),
                'pose': (-0.71983, 0.37106, 0.33145, 0.17737, 0.65857, 0.727895, 0.07055)
            },
            
            # scanning
            'one': {
                'joints': (),
                'pose': (-0.75000, -0.70000, 0.80000, 0.0, -0.70710678, 0.0, 0.70710678)
            },
            'two': {
                'joints': (),
                'pose': (-0.75000, 0.70000, 0.80000, 0.0, -0.70710678, 0.0, 0.70710678)
            },
            'three': {
                'joints': (),
                'pose': (-0.75000, 0.70000, 0.60000, 0.0, -0.70710678, 0.0, 0.70710678)
            },
            'four': {
                'joints': (),
                'pose': (-0.75000, -0.70000, 0.60000, 0.0, -0.70710678, 0.0, 0.70710678)
            },
            
            'five': {
                'joints': (),
                'pose': (-0.75000, -0.70000, 0.40000, 0.0, -0.70710678, 0.0, 0.70710678)
            },
            
            'six': {
                'joints': (),
                'pose': (-0.75000, 0.70000, 0.40000, 0.0, -0.70710678, 0.0, 0.70710678)
            },
            
            # testing simulation
            'p1': {
                'joints': (),
                'pose': (-0.75000, 0.70000, 0.80000, 0.5, -0.5, -0.5, 0.5)
            },
            'initial': {
                'joints': (),
                'pose': (0.17536, 0.43514, 0.28396, -0.16963, 0.98548, 0.00540, 0.00323)
            },
            'under': {
                'joints': (),
                'pose': (0.00068, 1.00176, -0.58394, -0.16963, 0.98548, 0.00540, 0.00323)
            },
            'under1': {
                'joints': (),
                'pose': (0.17461, 0.43615, 0.28621, -0.16950, 0.98540, 0.00600, 0.00140)
            },
            'under2': {
                'joints': (),
                'pose': (-0.51400, -0.51400, -0.76200, 0.924, -0.380, 0.0, 0.0)
            },
            
            # FSM positions
            'folded_fsm': {
                'joints': (),
                'pose': (-0.254, -0.273, 0.400, 0.498, 0.710, 0.286, 0.407)  # Raised Z from 0.401 to 0.550
            },
            'unfolded_fsm': {
                'joints': (),
                'pose': (0.411, -0.173, 0.850, 0.406, 0.577, 0.408, 0.580)
            }
        }
        
        # State variables
        self.goal_sent = False
        self.movement_done = False
        self.execution_status = False
        self.current_position_name = None
        self.emergency_stop = False
        self.end_effector_pose = None
        
        # Expected joint names in desired order
        self.expected_joint_names = [
            'arm_shoulder_pan_joint',
            'arm_shoulder_lift_joint',
            'arm_elbow_joint',
            'arm_wrist_1_joint',
            'arm_wrist_2_joint',
            'arm_wrist_3_joint'
        ]
        self.joint_indices = None
        
        # Timer for checking status
        self.timer = self.create_timer(0.5, self.check_status)
        
        self.get_logger().info("Position Sender Node initialized as a service.")
        self.get_logger().info(f"Available positions: {list(self.positions.keys())}")
        self.get_logger().info("Service available at: /send_position")
        self.get_logger().info("\033[1;32mUse: ros2 service call /send_position arm_control/srv/SendPosition \"{position_name: '<name>'}\"\033[0m")
        
    def joint_state_callback(self, msg):
        # Recompute mapping on every message because different joint_states sources
        # can have different joint ordering/lengths.
        joint_indices = []
        for expected_name in self.expected_joint_names:
            try:
                idx = msg.name.index(expected_name)
            except ValueError:
                return
            if idx >= len(msg.position):
                return
            joint_indices.append(idx)

        self.joint_indices = joint_indices
        self.current_joint_state = msg
        
    def end_effector_pose_callback(self, msg):
        self.end_effector_pose = msg
        
    def emergency_callback(self, msg):
        self.emergency_stop = msg.data
        
    def execution_status_callback(self, msg):
        """Callback for execution status updates."""
        self.execution_status = msg.data
        if self.emergency_stop:
            self.get_logger().info(f"Goal '{self.current_position_name}' canceled!")
            return
        if self.execution_status and self.goal_sent:
            self.movement_done = True
            self.get_logger().info(f"Position '{self.current_position_name}' reached successfully!")
            self.goal_sent = False
    
    def send_position_service_callback(self, request, response):
        """Service callback to send a requested position."""
        position_name = request.position_name
        
        self.get_logger().info(f"Received request to send position: '{position_name}'")
        
        success, message = self.send_position(position_name)
        
        response.success = success
        response.message = message
        
        return response
            
    def send_position(self, position_name):
        """Send a predefined position to the manipulator."""
        if self.emergency_stop:
            msg = "Cannot send position while emergency stop is active."
            self.get_logger().warn(msg)
            return False, msg
        
        if position_name not in self.positions:
            msg = f"Position '{position_name}' not found! Available positions: {list(self.positions.keys())}"
            self.get_logger().error(f"Position '{position_name}' not found!")
            self.get_logger().info(f"Available positions: {list(self.positions.keys())}")
            return False, msg
        
        position_data = self.positions[position_name]
        pose_data = position_data.get('pose')
        
        if pose_data is None:
            msg = f"No pose data defined for position '{position_name}'"
            self.get_logger().error(msg)
            return False, msg
        
        if len(pose_data) != 7:
            msg = f"Invalid pose data for '{position_name}': expected 7 values (x,y,z,qx,qy,qz,qw)"
            self.get_logger().error(msg)
            return False, msg

        if self.publisher_.get_subscription_count() == 0:
            msg = "No planner subscriber detected on /arm/goal_pose. Goal not sent."
            self.get_logger().warn(msg)
            return False, msg
            
        ####################################
        ## GOAL POSE PUBLISHING FOR PLANNER
        ####################################
        
        # Create and publish PoseStamped message
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'arm_base'
        msg.pose.position.x = pose_data[0]
        msg.pose.position.y = pose_data[1]
        msg.pose.position.z = pose_data[2]
        msg.pose.orientation.x = pose_data[3]
        msg.pose.orientation.y = pose_data[4]
        msg.pose.orientation.z = pose_data[5]
        msg.pose.orientation.w = pose_data[6]
        
        self.publisher_.publish(msg)        
        
        self.goal_sent = True
        self.movement_done = False
        self.current_position_name = position_name
        
        self.get_logger().info(f"→ Sending position '{position_name}' to planner...")
        self.get_logger().info(f"   Position: x={pose_data[0]:.3f}, y={pose_data[1]:.3f}, z={pose_data[2]:.3f}")
        self.get_logger().info(f"   Orientation: x={pose_data[3]:.3f}, y={pose_data[4]:.3f}, z={pose_data[5]:.3f}, w={pose_data[6]:.3f}")
        
        success_msg = f"Position '{position_name}' sent to planner successfully"
        return True, success_msg
    
    def check_status(self):
        """Periodic check of movement status."""
        if self.goal_sent and not self.movement_done:
            if not self.execution_status:
                # Still moving
                pass
            else:
                # Movement completed
                self.movement_done = True
    
    def add_position(self, name, joints=None, pose=None):
        """Add a new position to the available positions."""
        if pose is not None and len(pose) != 7:
            self.get_logger().error(f"Invalid pose data: expected 7 values (x,y,z,qx,qy,qz,qw)")
            return False
        
        if joints is not None and len(joints) != 6:
            self.get_logger().error(f"Invalid joint data: expected 6 values")
            return False
        
        self.positions[name] = {
            'joints': joints,
            'pose': pose
        }
        self.get_logger().info(f"✓ Position '{name}' added successfully")
        return True
    
    def list_positions(self):
        """List all available positions."""
        self.get_logger().info("=" * 50)
        self.get_logger().info("Available positions:")
        for name, data in self.positions.items():
            joints = data.get('joints', 'Not defined')
            pose = data.get('pose', 'Not defined')
            self.get_logger().info(f"  - {name}")
            if joints != 'Not defined':
                self.get_logger().info(f"      Joints: {joints}")
            if pose != 'Not defined':
                self.get_logger().info(f"      Pose: {pose}")
        self.get_logger().info("=" * 50)

def main(args=None):
    rclpy.init(args=args)
    
    node = PositionSenderNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
