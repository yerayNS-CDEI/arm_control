#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
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
        self.declare_parameter('planner_backend', 'legacy')
        self.planner_backend = str(self.get_parameter('planner_backend').value).strip().lower()
        
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        # Publishers for planner goals
        self.pose_goal_pub = self.create_publisher(PoseStamped, '/arm/goal_pose', 10)
        self.pose_goal_ptp_pub = self.create_publisher(PoseStamped, '/arm/goal_pose_ptp', 10)
        self.pose_goal_ompl_pub = self.create_publisher(PoseStamped, '/arm/goal_pose_ompl', 10)
        self.pose_goal_aps_pub = self.create_publisher(PoseStamped, '/arm/goal_pose_aps', 10)
        self.joint_goal_pub = self.create_publisher(JointState, '/arm/joint_goal', 10)
        self.joint_goal_ptp_pub = self.create_publisher(JointState, '/arm/joint_goal_ptp', 10)
        self.joint_goal_aps_pub = self.create_publisher(JointState, '/arm/joint_goal_aps', 10)
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
                'joints': (3.290, -2.007, -2.513, -0.647, -0.468, -1.048),
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
                'pose': (0.1749, 0.5879, -0.0058, -0.1679, 0.9753, -0.1413, -0.0240)
            },
            'under': {
                'joints': (1.346, -2.958, -1.727, -0.069, 1.607, 2.134),
                'pose': (0.315, 0.667, -0.700, -0.382, 0.923, -0.006, -0.027)
            },
            'under1': {
                'joints': (3.810, -2.978, -1.793, 0.103, 1.506, 1.417),
                'pose': (-0.681, -0.281, -0.700, -0.382, 0.923, -0.006, -0.027)
            },
            'under2': {
                'joints': (-5.73, 0.176, 1.426, -3.18, 4.714, -3.396),
                'pose': (-0.51400, -0.51400, -0.76200, 0.924, -0.380, 0.0, 0.0)
            },
            
            # FSM positions
            'folded_fsm': {
                'joints': (),
                'pose': (-0.36937, 0.38604, 0.00443, 0.6934, 0.6887, 0.1682, -0.1285)  # Raised Z from 0.401 to 0.550
            },
            'unfolded_fsm': {
                # Joints copied from 'custom' — a known-IK-valid forward-facing
                # configuration that produces an EE pose at ~(0.561, -0.173, 0.851)
                # with the same orientation as the target pose below.
                #
                # WHY JOINT VALUES: pose-goal planning to this target from a
                # behind-the-robot start (shoulder_pan ≈ π) requires APS to
                # find a ~180° shoulder swing through the column + plate +
                # camera + lidar mount workspace, which random sampling cannot
                # find in 30s.  Joint-space planning bypasses the seeded-IK
                # step entirely so APS gets an unambiguous goal and is far more
                # likely to find a clear corridor.
                #
                # SIDE-EFFECT: the final EE position is ~0.15m off the original
                # (0.411, -0.273, 0.850) target, but ExhaustiveScan re-positions
                # the arm during pre-approach so this offset is benign.
                'joints': (0.0, -1.104, -2.034, 0.0, 1.57, 2.8),
                'pose': (0.411, -0.273, 0.850, 0.406, 0.577, 0.408, 0.580)
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
        # OMPL pose goals: positions that have joint data but where a Cartesian pose goal
        # is preferred (the planner samples IK at the goal during search).
        self.ompl_pose_positions = {'unfolded', 'folded', 'under', 'under1', 'under2'}
        # Joint goals routed through APS (collision-aware) instead of Pilz PTP
        # (joint-space linear interpolation, NOT collision-aware).  unfolded_fsm
        # is here because PTP would sweep the upper arm through the column /
        # plate / camera / lidar mounts during the ~180° shoulder_pan rotation.
        self.aps_joint_positions = {'folded', 'unfolded', 'unfolded_fsm'}
        # APS pose goals: pose-only positions where (a) random IK sampling at the goal
        # fails for OMPL (orientation is too constrained — "Unable to sample goal tree")
        # and (b) PTP's joint-space linear interpolation collides during the sweep
        # (arm_upper_arm_link vs arm_plate_link / camera / os_sensor_mount).
        # moveit_planner_node now seeds IK from the current joint state before APS so the
        # goal joint config is known-valid, and APS plans collision-aware in joint space.
        self.aps_pose_positions = {'folded_fsm', 'unfolded_fsm'}
        self.joint_indices = None
        self.goal_marker_namespace = 'position_sender_goal_pose'
        
        # Timer for checking status
        self.timer = self.create_timer(0.5, self.check_status)
        
        self.get_logger().info("Position Sender Node initialized as a service.")
        self.get_logger().info(f"Available positions: {list(self.positions.keys())}")
        self.get_logger().info("Service available at: /send_position")
        self.get_logger().info(f"Planner backend mode: {self.planner_backend}")
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

    def publish_goal_pose_marker(self, position_name, pose_data):
        """Publish a pose marker with RGB axes so the goal frame is visible in RViz."""
        try:
            origin = np.array(pose_data[:3], dtype=float)
            rotation = R.from_quat(pose_data[3:])
        except ValueError as exc:
            self.get_logger().warn(
                f"Could not publish goal marker for '{position_name}': invalid pose/quaternion ({exc})"
            )
            return

        latest_tf_time = rclpy.time.Time().to_msg()
        axis_length = 0.12
        axis_endpoints = rotation.apply(np.eye(3) * axis_length) + origin

        origin_marker = Marker()
        origin_marker.header.frame_id = 'arm_base'
        origin_marker.header.stamp = latest_tf_time
        origin_marker.ns = self.goal_marker_namespace
        origin_marker.id = 0
        origin_marker.type = Marker.SPHERE
        origin_marker.action = Marker.ADD
        origin_marker.pose.position.x = float(origin[0])
        origin_marker.pose.position.y = float(origin[1])
        origin_marker.pose.position.z = float(origin[2])
        origin_marker.pose.orientation.w = 1.0
        origin_marker.scale.x = 0.035
        origin_marker.scale.y = 0.035
        origin_marker.scale.z = 0.035
        origin_marker.color = ColorRGBA(r=1.0, g=0.85, b=0.2, a=0.95)
        origin_marker.lifetime.sec = 0
        self.marker_pub.publish(origin_marker)

        axis_colors = (
            ColorRGBA(r=1.0, g=0.2, b=0.2, a=0.95),
            ColorRGBA(r=0.2, g=1.0, b=0.2, a=0.95),
            ColorRGBA(r=0.2, g=0.4, b=1.0, a=0.95),
        )

        start_point = Point(x=float(origin[0]), y=float(origin[1]), z=float(origin[2]))
        for axis_id, (endpoint, color) in enumerate(zip(axis_endpoints, axis_colors), start=1):
            axis_marker = Marker()
            axis_marker.header.frame_id = 'arm_base'
            axis_marker.header.stamp = latest_tf_time
            axis_marker.ns = self.goal_marker_namespace
            axis_marker.id = axis_id
            axis_marker.type = Marker.ARROW
            axis_marker.action = Marker.ADD
            axis_marker.points = [
                start_point,
                Point(x=float(endpoint[0]), y=float(endpoint[1]), z=float(endpoint[2]))
            ]
            axis_marker.scale.x = 0.01
            axis_marker.scale.y = 0.02
            axis_marker.scale.z = 0.025
            axis_marker.color = color
            axis_marker.lifetime.sec = 0
            self.marker_pub.publish(axis_marker)

        self.get_logger().info(f"Published goal pose marker for '{position_name}' on /goal_pose_marker")
            
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
        joint_data = position_data.get('joints')
        pose_data = position_data.get('pose')

        if self.planner_backend == 'moveit' and joint_data \
                and position_name not in self.ompl_pose_positions:
            return self.send_joint_goal(position_name, joint_data)
        
        if pose_data is None:
            msg = f"No pose data defined for position '{position_name}'"
            self.get_logger().error(msg)
            return False, msg
        
        if len(pose_data) != 7:
            msg = f"Invalid pose data for '{position_name}': expected 7 values (x,y,z,qx,qy,qz,qw)"
            self.get_logger().error(msg)
            return False, msg

        if self.planner_backend == 'moveit':
            if position_name in self.ompl_pose_positions:
                pose_goal_pub = self.pose_goal_ompl_pub
                goal_topic = '/arm/goal_pose_ompl'
                goal_label = 'OMPL pose goal'
            elif position_name in self.aps_pose_positions:
                pose_goal_pub = self.pose_goal_aps_pub
                goal_topic = '/arm/goal_pose_aps'
                goal_label = 'APS pose goal (seeded IK + collision-aware joint planning)'
            else:
                pose_goal_pub = self.pose_goal_ptp_pub
                goal_topic = '/arm/goal_pose_ptp'
                goal_label = 'PTP pose goal'
        else:
            pose_goal_pub = self.pose_goal_pub
            goal_topic = '/arm/goal_pose'
            goal_label = 'planner goal'

        if pose_goal_pub.get_subscription_count() == 0:
            msg = f"No planner subscriber detected on {goal_topic}. Goal not sent."
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
        
        pose_goal_pub.publish(msg)
        self.publish_goal_pose_marker(position_name, pose_data)
        
        self.goal_sent = True
        self.movement_done = False
        self.current_position_name = position_name
        
        self.get_logger().info(f"→ Sending position '{position_name}' as a {goal_label}...")
        self.get_logger().info(f"   Position: x={pose_data[0]:.3f}, y={pose_data[1]:.3f}, z={pose_data[2]:.3f}")
        self.get_logger().info(f"   Orientation: x={pose_data[3]:.3f}, y={pose_data[4]:.3f}, z={pose_data[5]:.3f}, w={pose_data[6]:.3f}")

        success_msg = f"Position '{position_name}' sent successfully via {goal_label}"
        return True, success_msg

    def send_joint_goal(self, position_name, joint_data):
        """Send a joint-space goal to the MoveIt planner."""
        if len(joint_data) != 6:
            msg = f"Invalid joint data for '{position_name}': expected 6 joint values"
            self.get_logger().error(msg)
            return False, msg

        if position_name in self.aps_joint_positions:
            joint_goal_pub = self.joint_goal_aps_pub
            goal_topic = '/arm/joint_goal_aps'
            planner_label = 'APS'
        else:
            joint_goal_pub = self.joint_goal_ptp_pub
            goal_topic = '/arm/joint_goal_ptp'
            planner_label = 'PTP'

        if joint_goal_pub.get_subscription_count() == 0:
            msg = f"No MoveIt subscriber detected on {goal_topic}. Joint goal not sent."
            self.get_logger().warn(msg)
            return False, msg

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.expected_joint_names)
        msg.position = [float(value) for value in joint_data]

        joint_goal_pub.publish(msg)

        self.goal_sent = True
        self.movement_done = False
        self.current_position_name = position_name

        joint_values_text = ", ".join(f"{value:.3f}" for value in msg.position)
        self.get_logger().info(f"→ Sending joint goal '{position_name}' via MoveIt {planner_label}...")
        self.get_logger().info(f"   Joints: [{joint_values_text}]")

        success_msg = f"Joint goal '{position_name}' sent to MoveIt {planner_label} successfully"
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
