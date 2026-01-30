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
        self.subscriptor_ = self.create_subscription(PoseStamped, 'end_effector_pose', self.end_effector_pose_callback, 10)
        self.execution_status_sub = self.create_subscription(Bool,'execution_status',self.execution_status_callback,10)
        self.emergency_sub = self.create_subscription(Bool, "emergency_stop", self.emergency_callback, qos)        
        self.create_subscription(JointState, "joint_states", self.joint_state_callback, 10)
        
        
        # Predefined positions (can be loaded from config file)
        self.positions = {
            'custom': {
                'joints': (0.0, -1.104, -2.034, 0.0, 1.57, 2.8),    # ONLY USED FOR DIPSLAYING PURPOSES
                'pose': (0.411, -0.173, 0.850, 0.406, 0.577, 0.408, 0.580)
            },
            'folded': {
                'joints': (3.1313, -1.6422, -2.4464, -2.7985, -0.6618, 0.2421),
                'pose': (-0.366, 0.269, 0.189, -0.327, 0.753, 0.564, -0.087)
            },
            'unfolded': {
                'joints': (3.291, -2.169, -1.523, -2.790, -0.367, 0.167),
                'pose': (-0.882, 0.152, 0.263, -0.193, 0.706, 0.661, -0.166)  # Will be computed if needed
            },
            'up': {
                'joints': (),
                'pose': (-0.286, 0.230, 0.572, -0.067, 0.562, 0.821, -0.071)
            },
            'down': {
                'joints': (),
                'pose': (-0.281, 0.209, 0.451, -0.105, 0.908, 0.404, -0.032)
            },
            'front': {
                'joints': (),
                'pose': (-0.287, 0.238, 0.530, -0.083, 0.702, 0.705, -0.06)
            },
            'left': {   
                'joints': (),
                'pose': (-0.388, 0.113, 0.501, -0.442, 0.549, 0.564,-0.430)
            },
            'right': {
                'joints': (),
                'pose': (-0.7725, 0.2310, 0.321, 0.17737, 0.65857, 0.727895, 0.07055)
            },
            
            # scanning
            'one': {
                'joints': (),
                'pose': (-0.6, -0.7, 0.8, 0.0, -0.70710678, 0.0, 0.70710678)
            },
            'two': {
                'joints': (),
                'pose': (-0.6, 0.7, 0.8, 0.0, -0.70710678, 0.0, 0.70710678)
            },
            'three': {
                'joints': (),
                'pose': (-0.6, 0.7, 0.6, 0.0, -0.70710678, 0.0, 0.70710678)
            },
            'four': {
                'joints': (),
                'pose': (-0.6, -0.7, 0.6, 0.0, -0.70710678, 0.0, 0.70710678)
            },
            
            'five': {
                'joints': (),
                'pose': (-0.6, -0.7, 0.4, 0.0, -0.70710678, 0.0, 0.70710678)
            },
            
            'six': {
                'joints': (),
                'pose': (-0.6, 0.7, 0.4, 0.0, -0.70710678, 0.0, 0.70710678)
            },
            
            # testing simulation
            'p1': {
                'joints': (),
                'pose': (-0.6, 0.7, 0.8, 0.5, -0.5, -0.5, 0.5)
            }
        }
        
        # State variables
        self.goal_sent = False
        self.movement_done = False
        self.execution_status = False
        self.current_position_name = None
        self.emergency_stop = False
        
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
        
        self.get_logger().info("Position Sender Node initialized.")
        self.get_logger().info(f"Available positions: {list(self.positions.keys())}")
        self.get_logger().info("Use: ros2 run arm_control position_sender <position_name>")
        
    def joint_state_callback(self, msg):
        # Generate index array mapping expected order to actual message order
        if self.joint_indices is None:
            self.joint_indices = []
            for expected_name in self.expected_joint_names:
                try:
                    idx = msg.name.index(expected_name)
                    self.joint_indices.append(idx)
                except ValueError:
                    self.get_logger().error(f"Joint '{expected_name}' not found in joint_states message")
                    self.joint_indices = None
                    return
        
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
            
    def send_position(self, position_name):
        """Send a predefined position to the manipulator."""
        if self.emergency_stop:
            self.get_logger().warn("Cannot send position while emergency stop is active.")
            return False
        
        if position_name not in self.positions:
            self.get_logger().error(f"Position '{position_name}' not found!")
            self.get_logger().info(f"Available positions: {list(self.positions.keys())}")
            return False
        
        position_data = self.positions[position_name]
        pose_data = position_data.get('pose')
        
        if pose_data is None:
            self.get_logger().error(f"No pose data defined for position '{position_name}'")
            return False
        
        if len(pose_data) != 7:
            self.get_logger().error(f"Invalid pose data for '{position_name}': expected 7 values (x,y,z,qx,qy,qz,qw)")
            return False
        
        current_position  = np.array([self.end_effector_pose.pose.position.x, self.end_effector_pose.pose.position.y, self.end_effector_pose.pose.position.z])
        dist_diff = pose_data[0:3] - current_position
        distance = np.linalg.norm(dist_diff)
            
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
        
        return True
    
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
    
    # Get command line arguments
    import sys
    node_args = remove_ros_args(sys.argv)[1:]  # node-only args (excluding script name)

    if len(node_args) > 0:
        position_name = node_args[0]
        
        if position_name == 'list':
            node.list_positions()
        else:
            node.send_position(position_name)
            
            # Spin until movement is done
            try:
                while rclpy.ok() and not node.movement_done:
                    rclpy.spin_once(node, timeout_sec=0.1)
                
                if node.movement_done:
                    node.get_logger().info("Movement completed. Shutting down.")
            except KeyboardInterrupt:
                node.get_logger().info("Interrupted by user.")
    else:
        node.get_logger().info("No position specified. Running in interactive mode.")
        node.get_logger().info("Type a position name to send it, 'list' to see all positions, or 'quit' to exit.")
        node.list_positions()
        
        # Interactive mode with input
        import threading
        
        def spin_node():
            try:
                rclpy.spin(node)
            except:
                pass
        
        # Start spinning in a separate thread
        spin_thread = threading.Thread(target=spin_node, daemon=True)
        spin_thread.start()
        
        try:
            while rclpy.ok():
                try:
                    user_input = input("\nEnter position name (or 'list'/'quit'): ").strip()
                    
                    if user_input.lower() in ['quit', 'exit', 'q']:
                        node.get_logger().info("Exiting interactive mode.")
                        break
                    elif user_input.lower() == 'list':
                        node.list_positions()
                    elif user_input:
                        success = node.send_position(user_input)
                        if success:
                            # Wait for movement to complete
                            import time
                            while not node.movement_done and rclpy.ok():
                                time.sleep(0.1)
                    else:
                        node.get_logger().warn("Empty input. Please enter a valid position name.")
                except EOFError:
                    break
                except KeyboardInterrupt:
                    node.get_logger().info("\nInterrupted by user.")
                    break
        except Exception as e:
            node.get_logger().error(f"Error in interactive mode: {e}")
        
        node.get_logger().info("Shutting down.")
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()