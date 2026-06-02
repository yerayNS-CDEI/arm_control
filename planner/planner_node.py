#!/usr/bin/env python3

import copy
from enum import Enum, auto
import math
import os
import threading
import time

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, ColorRGBA
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker
from ament_index_python.packages import get_package_share_directory
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import tf2_ros
from tf2_ros import TransformException

# Import planning functions
from planner.planner_lib.closed_form_algorithm import closed_form_algorithm
from planner.planner_lib.Astar3D import (
    dilate_obstacles,
    find_path,
    grid_to_world,
    world_to_grid_checked,
)
from scipy.spatial.transform import Rotation as R, Slerp
from scipy.ndimage import binary_erosion

# Import collision checking services
from arm_control.srv import CheckCollisionPose, CheckCollisionPoses
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

class CollisionCheckStatus(Enum):
    FREE = auto()
    COLLISION = auto()
    CHECK_FAILED = auto()

class IKSelectionOutcome(Enum):
    SUCCESS = auto()
    ALL_IN_COLLISION = auto()
    NO_VALID_SOLUTIONS = auto()
    CHECK_FAILED = auto()

class PlannerNode(Node):
    def __init__(self):
        super().__init__('planner_node')
        
        # Default to WARN so trajectory validation does not spend time formatting hot-loop logs.
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.WARN)
        
        # Declare and get joint prefix parameter
        self.declare_parameter('joint_prefix', 'arm_')
        self.joint_prefix = self.get_parameter('joint_prefix').get_parameter_value().string_value
        self.declare_parameter('visualize_goal_orientation', True)
        self.visualize_goal_orientation = self.get_parameter('visualize_goal_orientation').get_parameter_value().bool_value
        
        # Maximum number of IK solutions to test per waypoint (0 = test all 8 solutions)
        self.declare_parameter('max_ik_solutions_to_test', 1)
        self.max_ik_solutions_to_test = self.get_parameter('max_ik_solutions_to_test').get_parameter_value().integer_value

        # Collision validation visualization is expensive because each service call may trigger
        # marker publication and TF updates in the collision node. Keep it opt-in.
        self.declare_parameter('visualize_collision_checks', True)
        self.visualize_collision_checks = self.get_parameter('visualize_collision_checks').get_parameter_value().bool_value
        self.declare_parameter('log_performance_metrics', True)
        self.log_performance_metrics = self.get_parameter('log_performance_metrics').get_parameter_value().bool_value
        self.declare_parameter('collision_metrics_log_interval', 150)
        self.collision_metrics_log_interval = self.get_parameter('collision_metrics_log_interval').get_parameter_value().integer_value
        self.declare_parameter('cache_collision_checks', True)
        self.cache_collision_checks = self.get_parameter('cache_collision_checks').get_parameter_value().bool_value
        self.declare_parameter('collision_cache_round_digits', 4)
        self.collision_cache_round_digits = self.get_parameter('collision_cache_round_digits').get_parameter_value().integer_value
        self.declare_parameter('enable_batch_collision_checks', True)
        self.enable_batch_collision_checks = self.get_parameter('enable_batch_collision_checks').get_parameter_value().bool_value
        self.declare_parameter('hot_loop_info_logging', False)
        self.hot_loop_info_logging = self.get_parameter('hot_loop_info_logging').get_parameter_value().bool_value
        self.declare_parameter('publish_replanning_visualization', False)
        self.publish_replanning_visualization = self.get_parameter('publish_replanning_visualization').get_parameter_value().bool_value
        self.declare_parameter('visualize_dilated_grid', False)
        self.visualize_dilated_grid = self.get_parameter('visualize_dilated_grid').get_parameter_value().bool_value
        self.declare_parameter('validate_post_astar_tool0_path', False)
        self.validate_post_astar_tool0_path = self.get_parameter('validate_post_astar_tool0_path').get_parameter_value().bool_value
        self.declare_parameter('log_detailed_planning_diagnostics', False)
        self.log_detailed_planning_diagnostics = self.get_parameter('log_detailed_planning_diagnostics').get_parameter_value().bool_value
        self.declare_parameter('astar_collision_mode', 'dual')
        self.astar_collision_mode = self.get_parameter('astar_collision_mode').get_parameter_value().string_value
        if self.astar_collision_mode not in ['dual', 'wrist3', 'tool0_proxy']:
            self.get_logger().warn(
                f"Invalid astar_collision_mode '{self.astar_collision_mode}'. "
                "Must be one of ['dual', 'wrist3', 'tool0_proxy']. Defaulting to 'dual'."
            )
            self.astar_collision_mode = 'dual'

        # Number of neighbor layers to mark around collision cells during replanning
        # 0 = mark only the collision cell itself
        # 1 = mark the 6 axis-aligned neighbors plus the center cell
        # 2+ = mark the full cubic neighborhood with radius = num_layers
        self.declare_parameter('collision_avoidance_neighbor_layers', 1)
        self.collision_avoidance_neighbor_layers = self.get_parameter('collision_avoidance_neighbor_layers').get_parameter_value().integer_value

        # Mode parameter: 'arm' or 'full' — both use the same static oriented box from mobile_base_geometry.yaml
        self.declare_parameter('mode', 'full')
        self.mode = self.get_parameter('mode').get_parameter_value().string_value
        if self.mode not in ['arm', 'full']:
            self.get_logger().error(f"Invalid mode '{self.mode}'. Must be 'arm' or 'full'. Defaulting to 'full'.")
            self.mode = 'full'
        self.declare_parameter('simplify_mobile_base_collision_geometry', True)
        self.simplify_mobile_base_collision_geometry = (
            self.get_parameter('simplify_mobile_base_collision_geometry').get_parameter_value().bool_value
        )
        self.declare_parameter('simplified_mobile_base_box_inflation', [0.10, 0.10, 0.20])
        self.simplified_mobile_base_box_inflation = list(
            self.get_parameter('simplified_mobile_base_box_inflation').get_parameter_value().double_array_value
        )
        if len(self.simplified_mobile_base_box_inflation) != 3:
            self.get_logger().warn(
                "simplified_mobile_base_box_inflation must have 3 values. Using default [0.10, 0.10, 0.20]."
            )
            self.simplified_mobile_base_box_inflation = [0.10, 0.10, 0.20]

        # --- Parameters for the grid / reachability ---
        self.robot_name = 'ur10e'
        self.filename = "reachability_map_0.05_step_20_orientations"
        # self.filename = "reachability_map_0.1_step_20_orientations"
        fn_npy = f"{self.filename}.npy"
        self.cart_step = float(self.filename.split('_')[2])
        self.cart_min, self.cart_max = -1.6, 1.6
        self.base_path = os.path.join(get_package_share_directory('arm_control'), 'resource')
        self.reachability_map_fn=os.path.join(self.base_path, fn_npy)

        self.get_logger().info(f"Loading reachability map from: {self.reachability_map_fn}")
        self.reachability_map = np.load(self.reachability_map_fn, allow_pickle=True).item()
        self.radius = 1.35  # For UR10e (hardcoded!!)

        self.execution_complete = True
        self.goal_queue = []
        self.current_joint_state = None
        self.emergency_stop = False
        self.end_effector_pose = None
        self.i = 0  
        self._state_lock = threading.RLock()
        self._planning_active = False
        self._planning_thread = None
        self._plan_generation = 0
        self._reset_plan_metrics()

        # TF buffer and listener to query transforms dynamically
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Wrist-3-to-tool0 fixed transform (queried from TF tree)
        self._T_wrist3_tool0 = None
        self._T_tool0_wrist3 = None
        self._tf_retry_count = 0
        
        # Start async transform loading with timer (retries every 1 second)
        self._tf_retry_timer = self.create_timer(1.0, self._update_wrist3_tool0_transform)

        # Define cilinder parameters
        self.cyl_center_xy = (0.0, 0.0)   # (xc, yc)
        self.cyl_radius    = 0.3
        self.z_min, self.z_max  = -1.112, 1.3

        # Define mobile manipulator base footprint obstacle (XY corners in arm_base frame)
        # Kept for reference and visualisation; occupancy grid now uses the oriented box below.
        self.base_footprint_xy = [
            (-0.2747,  0.7544),
            (0.3478,  0.1323),
            (-0.1328, -0.3487),
            (-0.7553,  0.2733),
        ]
        self.base_z_min = -1.112
        self.base_z_max = -0.1

        # Arm-mode box geometry parameters (single source of truth: config/mobile_base_geometry.yaml).
        # All values are in arm_base frame convention; see YAML comments for details.
        self.declare_parameter('arm_mode_base_raw_box_dims', [0.8802, 0.6801, 1.0120])
        self.arm_mode_base_raw_box_dims = list(
            self.get_parameter('arm_mode_base_raw_box_dims').get_parameter_value().double_array_value
        )
        self.declare_parameter('arm_mode_base_box_center', [-0.2037, 0.2028, -0.6060])
        self.arm_mode_base_box_center = list(
            self.get_parameter('arm_mode_base_box_center').get_parameter_value().double_array_value
        )
        self.declare_parameter('arm_mode_base_box_rotation_z_deg', -45.0)
        self.arm_mode_base_box_rotation_z_deg = (
            self.get_parameter('arm_mode_base_box_rotation_z_deg').get_parameter_value().double_value
        )
        
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

        # Create grid space
        self.x_vals = np.arange(self.cart_min + self.cart_step / 2, self.cart_max, self.cart_step)
        self.y_vals = self.x_vals
        self.z_levels = sorted(self.reachability_map.keys())
        self.grid_size = len(self.x_vals)
        self.grid_shape = (self.grid_size, self.grid_size, len(self.z_levels))
        z_min = min(self.z_levels)
        z_max = max(self.z_levels)
        self.z_vals = np.linspace(z_min, z_max, len(self.z_levels))  # assumes uniform spacing
        
        self.get_logger().info(f"Grid configuration: shape={self.grid_shape}, cart_step={self.cart_step}m, z_levels={len(self.z_levels)}")
        
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # Create subscriber and publishers
        self.create_subscription(PoseStamped, '/arm/goal_pose', self.goal_callback, 10)
        # Subscribe to both namespaced and global topics to survive launch namespace drift.
        self.create_subscription(JointState, "joint_states", self.joint_state_callback, 10)
        self.create_subscription(JointState, "/joint_states", self.joint_state_callback, 10)
        self.create_subscription(Bool, "execution_status", self.execution_status_callback, 10)
        self.create_subscription(Bool, "/execution_status", self.execution_status_callback, 10)
        self.create_subscription(PoseStamped, "end_effector_pose", self.end_effector_pose_callback, 10)
        self.create_subscription(PoseStamped, "/end_effector_pose", self.end_effector_pose_callback, 10)
        self.emergency_sub = self.create_subscription(Bool, "emergency_stop", self.emergency_callback, qos)
        self.reset_sub = self.create_subscription(Bool, "/planner/reset", self.reset_callback, 10)
        self.trajectory_pub = self.create_publisher(JointTrajectory, 'planned_trajectory', 10)
        self.planner_goal_failed_pub = self.create_publisher(Bool, '/planner/goal_failed', 10)
        self.cyl_marker_pub = self.create_publisher(Marker, 'obstacle_markers', 10)
        self.wall_marker_pub = self.create_publisher(Marker, 'wall_marker', 10)
        self.goal_marker_pub = self.create_publisher(Marker, 'goal_pose_marker', 10)
        self.ee_path_marker_pub = self.create_publisher(Marker, 'ee_path_markers', 10)
        self.reachability_marker_pub = self.create_publisher(Marker, 'reachability_boundary_marker', qos)
        self.dilated_grid_marker_pub = self.create_publisher(Marker, 'dilated_grid_marker', qos)

        # Create collision checking service clients
        # Note: path_collision_checking node is launched with namespace 'collision'
        self.collision_check_client = self.create_client(CheckCollisionPose, '/collision/check_collision_pose')
        self.collision_check_batch_client = self.create_client(CheckCollisionPoses, '/collision/check_collision_poses')
        
        # Collision checking configuration
        self.declare_parameter('enable_collision_checking', True)
        self.enable_collision_checking = self.get_parameter('enable_collision_checking').get_parameter_value().bool_value
        
        if self.enable_collision_checking:
            self.get_logger().info("⚠️  Collision checking ENABLED - may cause delays or timeouts in callbacks")
        else:
            self.get_logger().info("Collision checking DISABLED - planner will use closest IK solutions")
        
        # Collision service retry logic (handles race conditions with robot_description loading)
        self._collision_service_available = False
        self._collision_service_check_count = 0
        self._collision_service_max_retries = 15  # 15 attempts * 2 seconds = 30 seconds max wait
        self._collision_service_check_interval = 2.0  # seconds between checks
        
        if self.enable_collision_checking:
            self._collision_check_timer = self.create_timer(
                self._collision_service_check_interval, 
                self._check_collision_service_periodic
            )
        else:
            self._collision_service_available = False  # Explicitly disabled

        self._reachability_boundary_pts = self._compute_reachability_boundary()
        self._reachability_timer = self.create_timer(2.0, self._publish_reachability_once)
        self._goal_dispatch_timer = self.create_timer(0.05, self._process_goal_queue)

        self.get_logger().info("Planner node initialized and waiting for goal poses...")

    def _check_collision_service_periodic(self):
        """
        Periodically check collision service availability with retries.
        Handles race conditions where collision node starts after planner.
        """
        if self._collision_service_available:
            # Already connected, cancel timer
            return
        
        self._collision_service_check_count += 1
        
        if self.collision_check_client.service_is_ready():
            # Service is now available!
            self._collision_service_available = True
            self._collision_check_timer.cancel()
            self.get_logger().info(
                f"✅ Collision checking service connected: /collision/check_collision_pose "
                f"(found after {self._collision_service_check_count} attempts, "
                f"{self._collision_service_check_count * self._collision_service_check_interval:.1f}s)"
            )
        else:
            # Service not yet available
            if self._collision_service_check_count <= 3:
                # Be quiet for first few attempts (normal startup)
                self.get_logger().debug(
                    f"Waiting for collision service... (attempt {self._collision_service_check_count}/"
                    f"{self._collision_service_max_retries})"
                )
            elif self._collision_service_check_count >= self._collision_service_max_retries:
                # Max retries reached - give up and warn user
                self._collision_check_timer.cancel()
                self.get_logger().warn(
                    f"⚠️  Collision checking service NOT available at /collision/check_collision_pose\n"
                    f"   Waited {self._collision_service_check_count * self._collision_service_check_interval:.1f}s "
                    f"({self._collision_service_check_count} attempts) but service not found.\n"
                    f"   Planner will use closest IK solutions without collision validation.\n"
                    f"   \n"
                    f"   To enable collision checking, launch:\n"
                    f"     ros2 launch arm_control abstract_robot.launch.py\n"
                    f"   or\n"
                    f"     ros2 launch arm_control collision_view_ur.launch.py ur_type:=ur10e tf_prefix:=collision_"
                )
            else:
                # Still waiting, log periodically
                if self._collision_service_check_count % 5 == 0:
                    self.get_logger().info(
                        f"Still waiting for collision service... (attempt {self._collision_service_check_count}/"
                        f"{self._collision_service_max_retries}, elapsed: "
                        f"{self._collision_service_check_count * self._collision_service_check_interval:.1f}s)"
                    )

    def _update_wrist3_tool0_transform(self):
        """
        Query the TF tree for the transform from arm_wrist_3_link to arm_tool0.
        Updates self._T_wrist3_tool0 and self._T_tool0_wrist3 matrices.
        
        Uses asynchronous retry pattern with timer - does not block initialization.
        No fallback to hardcoded values for safety.
        """
        from_frame = f'{self.joint_prefix}wrist_3_link'
        to_frame = f'{self.joint_prefix}tool0'
        
        try:
            # Non-blocking check - if not available, timer will retry
            if self.tf_buffer.can_transform(from_frame, to_frame, rclpy.time.Time(), rclpy.duration.Duration(seconds=0.1)):
                # Query the transform FROM tool0 TO wrist_3 (this gives us where tool0 is in wrist_3 frame)
                # We swap the order: lookup_transform(wrist_3, tool0) returns T_wrist3_tool0
                transform = self.tf_buffer.lookup_transform(
                    from_frame,  # target frame: wrist_3 
                    to_frame,  # source frame: tool0
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5)
                )
                
                # Extract translation and rotation
                tx = transform.transform.translation.x
                ty = transform.transform.translation.y
                tz = transform.transform.translation.z
                
                qx = transform.transform.rotation.x
                qy = transform.transform.rotation.y
                qz = transform.transform.rotation.z
                qw = transform.transform.rotation.w
                
                # Build transformation matrix wrist_3 -> tool0
                rot = R.from_quat([qx, qy, qz, qw])
                self._T_wrist3_tool0 = np.eye(4)
                self._T_wrist3_tool0[:3, :3] = rot.as_matrix()
                self._T_wrist3_tool0[:3, 3] = [tx, ty, tz]
                
                # Compute inverse transform tool0 -> wrist_3
                self._T_tool0_wrist3 = np.linalg.inv(self._T_wrist3_tool0)
                
                self.get_logger().info(
                    f"✓ Successfully loaded transform {from_frame} -> {to_frame}: "
                    f"translation = [{tx:.4f}, {ty:.4f}, {tz:.4f}], "
                    f"rotation = [{qx:.3f}, {qy:.3f}, {qz:.3f}, {qw:.3f}]"
                )
                
                # Cancel retry timer if it exists
                if hasattr(self, '_tf_retry_timer'):
                    self._tf_retry_timer.cancel()
                return
            else:
                # Transform not yet available - will retry via timer
                if not hasattr(self, '_tf_retry_count'):
                    self._tf_retry_count = 0
                
                self._tf_retry_count += 1
                
                if self._tf_retry_count <= 5:
                    self.get_logger().warn(
                        f"Transform {from_frame} -> {to_frame} not yet available. "
                        f"Waiting for TF tree... (attempt {self._tf_retry_count})"
                    )
                elif self._tf_retry_count % 10 == 0:
                    # Log available frames to help debug
                    all_frames = self.tf_buffer.all_frames_as_yaml()
                    self.get_logger().error(
                        f"Transform still not available after {self._tf_retry_count} attempts. "
                        f"Available frames:\n{all_frames}"
                    )
                
        except (TransformException, tf2_ros.LookupException, 
                tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
                    if not hasattr(self, '_tf_retry_count'):
                        self._tf_retry_count = 0
            
                    self._tf_retry_count += 1
            
                    if self._tf_retry_count <= 5 or self._tf_retry_count % 10 == 0:
                        self.get_logger().warn(
                    f"TF lookup exception (attempt {self._tf_retry_count}): {ex}"
                        )

    def _compute_reachability_boundary(self):
        """Build the outer-shell voxels of the reachability space using morphological erosion.

        A voxel is considered a boundary voxel if it is reachable AND at least one of its
        6-connected neighbours is not reachable (or is outside the grid).  This is equivalent
        to: boundary = reachable AND NOT eroded(reachable).
        """
        reachable_3d = np.zeros(self.grid_shape, dtype=bool)
        for k, z in enumerate(self.z_levels):
            reachable_3d[:, :, k] = np.asarray(self.reachability_map[z]) > 0

        # binary_erosion with border_value=0 treats out-of-bounds as empty,
        # so edge voxels of the reachable region are always kept as boundary.
        interior = binary_erosion(reachable_3d, border_value=0)
        boundary_mask = reachable_3d & ~interior

        ii, jj, kk = np.where(boundary_mask)
        pts = [
            (float(self.x_vals[i]), float(self.y_vals[j]), float(self.z_vals[k]))
            for i, j, k in zip(ii, jj, kk)
        ]
        self.get_logger().info(f"Reachability boundary pre-computed: {len(pts)} surface voxels.")
        return pts

    def _publish_reachability_once(self):
        """One-shot timer callback: publish the reachability boundary marker then cancel."""
        self.publish_reachability_boundary_marker()
        self._reachability_timer.cancel()

    def publish_reachability_boundary_marker(self):
        """Publish the pre-computed reachability boundary as a SPHERE_LIST marker in RViz."""
        if not self._reachability_boundary_pts:
            self.get_logger().warn("Reachability boundary is empty — nothing to publish.")
            return

        m = Marker()
        m.header.frame_id = "arm_base"
        m.header.stamp = rclpy.time.Time().to_msg()
        m.ns = "reachability_boundary"
        m.id = 0
        m.type = Marker.SPHERE_LIST
        m.action = Marker.ADD

        # Each sphere is slightly smaller than the voxel step for a clean non-overlapping look
        scale = float(self.cart_step) * 0.8
        m.scale.x = scale
        m.scale.y = scale
        m.scale.z = scale

        # Semi-transparent blue/cyan shell
        m.color.r = 0.2
        m.color.g = 0.6
        m.color.b = 1.0
        m.color.a = 0.35

        m.points = [Point(x=x, y=y, z=z) for (x, y, z) in self._reachability_boundary_pts]
        m.lifetime.sec = 0  # Persist in RViz until deleted

        self.reachability_marker_pub.publish(m)
        self.get_logger().info(
            f"Published reachability boundary marker with {len(m.points)} points "
            f"on topic 'reachability_boundary_marker'."
        )

    def publish_dilated_grid_marker(self, occupancy_grid_dilated, frame_id="arm_base"):
        """Publish the static dilated occupancy grid used as the A* base grid."""
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = rclpy.time.Time().to_msg()
        marker.ns = "dilated_grid"
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = float(self.cart_step)
        marker.scale.y = float(self.cart_step)
        marker.scale.z = float(self.cart_step)
        marker.color.r = 1.0
        marker.color.g = 0.2
        marker.color.b = 0.2
        marker.color.a = 0.18
        marker.lifetime.sec = 0

        occupied_indices = np.argwhere(occupancy_grid_dilated == 1)
        marker.points = [
            Point(
                x=float(self.x_vals[i]),
                y=float(self.y_vals[j]),
                z=float(self.z_vals[k]),
            )
            for i, j, k in occupied_indices
        ]

        self.dilated_grid_marker_pub.publish(marker)
        self.get_logger().info(
            f"Published dilated grid marker with {len(marker.points)} occupied voxels "
            f"on topic 'dilated_grid_marker'."
        )
    
    def publish_collision_waypoints_markers(self, collision_waypoints_dict):
        """Publish collision waypoints as red spheres in RViz on goal_pose_marker topic.
        Uses individual SPHERE markers (not SPHERE_LIST) to match gap_boundaries style.
        
        Args:
            collision_waypoints_dict: Dict mapping grid_idx (i,j,k) -> num_layers
        """
        now = self.get_clock().now().to_msg()
        
        if not collision_waypoints_dict:
            # Clear all existing markers using DELETEALL
            clear_marker = Marker()
            clear_marker.header.frame_id = "arm_base"
            clear_marker.header.stamp = now
            clear_marker.ns = "collision_waypoints"
            clear_marker.id = 0
            clear_marker.action = Marker.DELETEALL
            self.goal_marker_pub.publish(clear_marker)
            self.get_logger().info("🔴 Cleared collision waypoint markers")
            return
        
        # Publish individual SPHERE markers (like gap_boundaries does)
        scale = float(self.cart_step) * 2.5  # Larger for better visibility
        sample_positions = []
        
        for marker_id, grid_idx in enumerate(collision_waypoints_dict.keys()):
            i, j, k = grid_idx
            world_pos = grid_to_world(i, j, k, self.x_vals, self.y_vals, self.z_vals)
            
            m = Marker()
            m.header.frame_id = "arm_base"
            m.header.stamp = now
            m.ns = "collision_waypoints"
            m.id = marker_id
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            
            # Position
            m.pose.position.x = float(world_pos[0])
            m.pose.position.y = float(world_pos[1])
            m.pose.position.z = float(world_pos[2])
            m.pose.orientation.w = 1.0
            
            # Size
            m.scale.x = scale
            m.scale.y = scale
            m.scale.z = scale
            
            # Bright red color
            m.color.r = 1.0
            m.color.g = 0.0
            m.color.b = 0.0
            m.color.a = 0.9
            
            m.lifetime.sec = 0  # Persist until deleted
            
            self.goal_marker_pub.publish(m)
            
            # Small delay to avoid flooding (1ms between markers)
            time.sleep(0.001)
            
            # Log first 3 positions for debugging
            if len(sample_positions) < 3:
                sample_positions.append(f"grid{grid_idx}→world({world_pos[0]:.3f},{world_pos[1]:.3f},{world_pos[2]:.3f})")
        
        num_subscribers = self.goal_marker_pub.get_subscription_count()
        self.get_logger().info(
            f"🔴 Published {len(collision_waypoints_dict)} individual collision waypoint spheres:\n"
            f"   Topic: goal_pose_marker ({num_subscribers} subscriber(s))\n"
            f"   Frame: arm_base, Namespace: collision_waypoints, Scale: {scale:.3f}m\n"
            f"   Samples: {sample_positions}"
        )

    def clear_goal_and_path_markers(self):
        """Clear stale goal/path markers before planning a new goal."""
        now = self.get_clock().now().to_msg()

        clear_goal = Marker()
        clear_goal.header.frame_id = "arm_base"
        clear_goal.header.stamp = now
        clear_goal.action = Marker.DELETEALL
        self.goal_marker_pub.publish(clear_goal)

        clear_path = Marker()
        clear_path.header.frame_id = "arm_base"
        clear_path.header.stamp = now
        clear_path.action = Marker.DELETEALL
        self.ee_path_marker_pub.publish(clear_path)
        
    def publish_wall(self):
        """Publish a wall marker for RViz visualization."""
        marker = Marker()
        marker.header.frame_id = "map"  # Change to your robot's base frame if different
        marker.header.stamp = rclpy.time.Time().to_msg()
        marker.ns = "obstacles"
        marker.id = 1
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        # Position: center of wall
        marker.pose.position.x = -2.5
        marker.pose.position.y = 2.0
        marker.pose.position.z = 2.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        
        # Scale: width, depth, and height
        marker.scale.x = 0.1  # thickness in x
        marker.scale.y = 8.0  # width in y
        marker.scale.z = 4.0  # height
        
        # Color: semi-transparent blue
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0  # opaque
        
        marker.lifetime.sec = 0  # 0 means it persists until deleted
         
        self.wall_marker_pub.publish(marker)
        self.get_logger().info(f"Published wall marker at (2.5, 2.0) with thickness 0.1, width 4.0, and height 4.0")
        
    def publish_goal_marker(self, pose_data):
        x, y, z = pose_data[0], pose_data[1], pose_data[2]
        quat = (pose_data[3], pose_data[4], pose_data[5], pose_data[6])

        # Sphere marker at the goal position (stores orientation too if you pass quat)
        m_sphere = self._make_sphere_marker(
            x, y, z,
            frame_id="arm_base",
            ns="goal_pose",
            mid=0,
            rgb=(0.0, 1.0, 0.0),
            scale=0.05,
            quat=quat
        )
        m_sphere.lifetime.sec = 0
        self.goal_marker_pub.publish(m_sphere)

        # Optional axes visualization
        if self.visualize_goal_orientation:
            m_axes = self._make_axes_marker(
                x, y, z,
                quat=quat,
                frame_id="arm_base",
                ns="goal_pose",
                mid=1,
                axis_len=0.12,
                axis_width=0.01
            )
            m_axes.lifetime.sec = 0
            self.goal_marker_pub.publish(m_axes)
    
    def _publish_gap_boundary_markers(self, start_pos, goal_pos, collision_waypoints_dict=None):
        """Publish markers showing the current A* search boundaries (gap start/goal) 
        and collision waypoints (red spheres) with their marked neighbors (orange spheres).
        
        Args:
            start_pos: Start boundary position [x, y, z]
            goal_pos: Goal boundary position [x, y, z]
            collision_waypoints_dict: Dict of collision waypoint grid indices {(i,j,k): num_layers}
        
        Visualization:
            - Blue sphere (ID 0): Start boundary
            - Orange sphere (ID 1): Goal boundary  
            - Red spheres (IDs 100+): Center collision waypoint cells
            - Orange spheres (IDs 100+): Neighbor cells marked as obstacles (if num_layers > 0)
        """
        now = rclpy.time.Time().to_msg()
        
        # Start boundary marker (blue sphere) - ID 0
        m_start = Marker()
        m_start.header.frame_id = "arm_base"
        m_start.header.stamp = now
        m_start.ns = "gap_boundaries"
        m_start.id = 0
        m_start.type = Marker.SPHERE
        m_start.action = Marker.ADD
        m_start.pose.position.x = float(start_pos[0])
        m_start.pose.position.y = float(start_pos[1])
        m_start.pose.position.z = float(start_pos[2])
        m_start.pose.orientation.w = 1.0
        m_start.scale.x = 0.08
        m_start.scale.y = 0.08
        m_start.scale.z = 0.08
        m_start.color.r = 0.0
        m_start.color.g = 0.5
        m_start.color.b = 1.0
        m_start.color.a = 0.8
        m_start.lifetime.sec = 0
        self.goal_marker_pub.publish(m_start)
        
        # Goal boundary marker (orange sphere) - ID 1
        m_goal = Marker()
        m_goal.header.frame_id = "arm_base"
        m_goal.header.stamp = now
        m_goal.ns = "gap_boundaries"
        m_goal.id = 1
        m_goal.type = Marker.SPHERE
        m_goal.action = Marker.ADD
        m_goal.pose.position.x = float(goal_pos[0])
        m_goal.pose.position.y = float(goal_pos[1])
        m_goal.pose.position.z = float(goal_pos[2])
        m_goal.pose.orientation.w = 1.0
        m_goal.scale.x = 0.08
        m_goal.scale.y = 0.08
        m_goal.scale.z = 0.08
        m_goal.color.r = 1.0
        m_goal.color.g = 0.5
        m_goal.color.b = 0.0
        m_goal.color.a = 0.8
        m_goal.lifetime.sec = 0
        self.goal_marker_pub.publish(m_goal)
        
        # Collision waypoint markers (red spheres) - IDs starting from 100
        if collision_waypoints_dict:
            scale = 0.04  # Much smaller than boundary markers (0.08) for less visual clutter
            neighbor_scale = 0.03  # Even smaller for neighbor cells
            marker_id = 100
            
            for grid_idx, num_layers in collision_waypoints_dict.items():
                i, j, k = grid_idx
                world_pos = grid_to_world(i, j, k, self.x_vals, self.y_vals, self.z_vals)
                
                # Publish center collision waypoint (bright red)
                m = Marker()
                m.header.frame_id = "arm_base"
                m.header.stamp = now
                m.ns = "gap_boundaries"
                m.id = marker_id
                m.type = Marker.SPHERE
                m.action = Marker.ADD
                
                m.pose.position.x = float(world_pos[0])
                m.pose.position.y = float(world_pos[1])
                m.pose.position.z = float(world_pos[2])
                m.pose.orientation.w = 1.0
                
                m.scale.x = scale
                m.scale.y = scale
                m.scale.z = scale
                
                # Bright red color for center cell
                m.color.r = 1.0
                m.color.g = 0.0
                m.color.b = 0.0
                m.color.a = 0.9
                
                m.lifetime.sec = 0
                self.goal_marker_pub.publish(m)
                marker_id += 1
                
                # Publish neighbor cells if num_layers > 0 (orange/yellow)
                if num_layers > 0:
                    for di in range(-num_layers, num_layers + 1):
                        for dj in range(-num_layers, num_layers + 1):
                            for dk in range(-num_layers, num_layers + 1):
                                # Skip center cell (already published)
                                if di == 0 and dj == 0 and dk == 0:
                                    continue
                                
                                ni, nj, nk = i + di, j + dj, k + dk
                                
                                # Check bounds
                                if (0 <= ni < len(self.x_vals) and
                                    0 <= nj < len(self.y_vals) and
                                    0 <= nk < len(self.z_vals)):
                                    
                                    neighbor_world_pos = grid_to_world(ni, nj, nk, self.x_vals, self.y_vals, self.z_vals)
                                    
                                    m_neighbor = Marker()
                                    m_neighbor.header.frame_id = "arm_base"
                                    m_neighbor.header.stamp = now
                                    m_neighbor.ns = "gap_boundaries"
                                    m_neighbor.id = marker_id
                                    m_neighbor.type = Marker.SPHERE
                                    m_neighbor.action = Marker.ADD
                                    
                                    m_neighbor.pose.position.x = float(neighbor_world_pos[0])
                                    m_neighbor.pose.position.y = float(neighbor_world_pos[1])
                                    m_neighbor.pose.position.z = float(neighbor_world_pos[2])
                                    m_neighbor.pose.orientation.w = 1.0
                                    
                                    m_neighbor.scale.x = neighbor_scale
                                    m_neighbor.scale.y = neighbor_scale
                                    m_neighbor.scale.z = neighbor_scale
                                    
                                    # Orange color for neighbor cells (less prominent)
                                    m_neighbor.color.r = 1.0
                                    m_neighbor.color.g = 0.5
                                    m_neighbor.color.b = 0.0
                                    m_neighbor.color.a = 0.6
                                    
                                    m_neighbor.lifetime.sec = 0
                                    self.goal_marker_pub.publish(m_neighbor)
                                    marker_id += 1
            
            # Delete any markers beyond what we just published
            for del_id in range(marker_id, 2000):  # Increased upper limit for neighbors
                clear_m = Marker()
                clear_m.header.frame_id = "arm_base"
                clear_m.header.stamp = now
                clear_m.ns = "gap_boundaries"
                clear_m.id = del_id
                clear_m.action = Marker.DELETE
                self.goal_marker_pub.publish(clear_m)
            
            # Count total markers published (collision centers + neighbors)
            total_markers = marker_id - 100
            self.get_logger().info(
                f"🔴 Visualized {len(collision_waypoints_dict)} collision waypoints ({total_markers} total markers: red=centers, orange=neighbors)"
            )
            # time.sleep(5.0)  # DEBUG: pause so RViz markers are visible before next iteration
        else:
            # Clear ALL collision markers (IDs 100-1999) when dict is empty
            self.get_logger().info("🗑️  Clearing all collision markers (dict is empty)")
            for marker_id in range(100, 2000):
                clear_m = Marker()
                clear_m.header.frame_id = "arm_base"
                clear_m.header.stamp = now
                clear_m.ns = "gap_boundaries"
                clear_m.id = marker_id
                clear_m.action = Marker.DELETE
                self.goal_marker_pub.publish(clear_m)
        
    def publish_ee_path_markers(self, path_world, frame_id="map", current_ee_xyz=None, goal_xyz=None,
                                ns="ee_path", id_offset=0, rgba=(0.0, 0.0, 1.0, 1.0),
                                z_offset=0.0, line_width=0.01, sphere_diam=0.03):
        """Publish EE path as spheres + a line strip connecting them."""
        now = rclpy.time.Time().to_msg()

        # --- Waypoints as SPHERE_LIST ---
        spheres = Marker()
        spheres.header.frame_id = frame_id
        spheres.header.stamp = now
        spheres.ns = ns
        spheres.id = id_offset
        spheres.type = Marker.SPHERE_LIST
        spheres.action = Marker.ADD

        # Sphere diameter
        spheres.scale.x = float(sphere_diam)
        spheres.scale.y = float(sphere_diam)
        spheres.scale.z = float(sphere_diam)

        # Color
        spheres.color.r = float(rgba[0])
        spheres.color.g = float(rgba[1])
        spheres.color.b = float(rgba[2])
        spheres.color.a = float(rgba[3])

        spheres.points = []
        for (x, y, z) in path_world:
            spheres.points.append(Point(x=float(x), y=float(y), z=float(z) + float(z_offset)))

        spheres.lifetime.sec = 0

        # --- Line connecting waypoints (LINE_STRIP) ---
        line = Marker()
        line.header.frame_id = frame_id
        line.header.stamp = now
        line.ns = ns
        line.id = id_offset + 1
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD

        # Line width
        line.scale.x = float(line_width)

        # Color
        line.color.r = float(rgba[0])
        line.color.g = float(rgba[1])
        line.color.b = float(rgba[2])
        line.color.a = float(rgba[3])

        line.points = []
        for (x, y, z) in path_world:
            line.points.append(Point(x=float(x), y=float(y), z=float(z) + float(z_offset)))

        line.lifetime.sec = 0

        self.ee_path_marker_pub.publish(spheres)
        self.ee_path_marker_pub.publish(line)

        # Current EE (green)
        if current_ee_xyz is not None:
            cx, cy, cz = current_ee_xyz
            m_cur = self._make_sphere_marker(cx, cy, cz, frame_id, ns, id_offset + 2, rgb=(0.0, 1.0, 0.0), scale=0.06)
            self.ee_path_marker_pub.publish(m_cur)

        # Goal (red)
        if goal_xyz is not None:
            gx, gy, gz = goal_xyz
            m_goal = self._make_sphere_marker(gx, gy, gz, frame_id, ns, id_offset + 3, rgb=(1.0, 0.0, 0.0), scale=0.06)
            self.ee_path_marker_pub.publish(m_goal)
        
    def _make_sphere_marker(self, x, y, z, frame_id, ns, mid, rgb, scale=0.05, quat=None):
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = rclpy.time.Time().to_msg()
        m.ns = ns
        m.id = mid
        m.type = Marker.SPHERE
        m.action = Marker.ADD

        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.position.z = float(z)
        
        if quat is not None:
            m.pose.orientation.x = float(quat[0])
            m.pose.orientation.y = float(quat[1])
            m.pose.orientation.z = float(quat[2])
            m.pose.orientation.w = float(quat[3])
        else:
            m.pose.orientation.w = 1.0

        m.scale.x = scale
        m.scale.y = scale
        m.scale.z = scale

        m.color.r = float(rgb[0])
        m.color.g = float(rgb[1])
        m.color.b = float(rgb[2])
        m.color.a = 1.0

        return m
    
    def _make_color(self, r, g, b, a=1.0):
        c = ColorRGBA()
        c.r = float(r)
        c.g = float(g)
        c.b = float(b)
        c.a = float(a)
        return c
    
    def _make_axes_marker(self, x, y, z, quat, frame_id, ns, mid, axis_len=0.12, axis_width=0.01):
        """
        Draw a coordinate triad at (x,y,z) using LINE_LIST:
        X axis = red, Y axis = green, Z axis = blue
        """
        r = R.from_quat([quat[0], quat[1], quat[2], quat[3]])

        # Unit axes in local frame, rotated to world frame
        ex = r.apply([axis_len, 0.0, 0.0])
        ey = r.apply([0.0, axis_len, 0.0])
        ez = r.apply([0.0, 0.0, axis_len])

        origin = Point(x=float(x), y=float(y), z=float(z))
        px = Point(x=float(x + ex[0]), y=float(y + ex[1]), z=float(z + ex[2]))
        py = Point(x=float(x + ey[0]), y=float(y + ey[1]), z=float(z + ey[2]))
        pz = Point(x=float(x + ez[0]), y=float(y + ez[1]), z=float(z + ez[2]))

        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = rclpy.time.Time().to_msg()
        m.ns = ns
        m.id = mid
        m.type = Marker.LINE_LIST
        m.action = Marker.ADD

        # LINE_LIST uses scale.x as line width
        m.scale.x = float(axis_width)

        # Points come in pairs (start,end) per segment
        m.points = [
            origin, px,  # X axis
            origin, py,  # Y axis
            origin, pz,  # Z axis
        ]

        # For LINE_LIST, you can specify per-vertex colors
        m.colors = []
        # X axis (red)
        m.colors.append(self._make_color(1.0, 0.0, 0.0, 1.0)); m.colors.append(self._make_color(1.0, 0.0, 0.0, 1.0))
        # Y axis (green)
        m.colors.append(self._make_color(0.0, 1.0, 0.0, 1.0)); m.colors.append(self._make_color(0.0, 1.0, 0.0, 1.0))
        # Z axis (blue)
        m.colors.append(self._make_color(0.0, 0.0, 1.0, 1.0)); m.colors.append(self._make_color(0.0, 0.0, 1.0, 1.0))

        m.pose.orientation.w = 1.0
        return m
    
    def _prune_path_endpoints(self, path_world, current_pos, goal_pos, tol_m=0.02, log=True, return_trim=False):
        """
        Single-pass endpoint pruning + allow pruning down to empty.

        Rules:
        1) Start-neighbor rule: drop w1 if cur->w2 is significantly closer than cur->w1
        2) End-neighbor rule:   drop wn if goal->w(n-1) is significantly closer than goal->wn
        3) Progress rule start: drop w1 if w1 is farther from goal than cur is (w1 is "behind")
        4) Progress rule end:   drop wn if wn is farther from cur than goal is (wn is "beyond")
        5) Start-detour rule: drop w1 if cur->w2 is shorter than (cur->w1 + w1->w2)
        6) End-detour rule: drop wn if w(n-1)->goal is shorter than (w(n-1)->wn + wn->goal)
        7) If 1 waypoint remains: drop it if direct cur<->goal is better than using it from both sides
        """
        if not path_world:
            if log:
                self.get_logger().info("[prune] input path empty -> return []")
            return ([], 0, 0) if return_trim else []

        pw = [(float(x), float(y), float(z)) for (x, y, z) in path_world]
        start_trim = 0
        end_trim = 0

        def d2(a, b):
            dx = a[0] - b[0]
            dy = a[1] - b[1]
            dz = a[2] - b[2]
            return dx*dx + dy*dy + dz*dz

        def d(a, b):
            # Actual distance (not squared)
            import math
            return math.sqrt(d2(a, b))

        tol2 = float(tol_m) * float(tol_m)
        cur = (float(current_pos[0]), float(current_pos[1]), float(current_pos[2]))
        goal = (float(goal_pos[0]), float(goal_pos[1]), float(goal_pos[2]))

        before = len(pw)

        # ---------- START pruning ----------
        if len(pw) >= 2:
            w1, w2 = pw[0], pw[1]

            # Neighbor rule
            if d2(cur, w2) + tol2 < d2(cur, w1):
                if log:
                    self.get_logger().info(
                        "[prune] start neighbor: drop w1 | "
                        f"cur={cur} goal={goal} w1={w1} w2={w2} | "
                        f"d2(cur,w1)={d2(cur,w1):.4f} d2(cur,w2)={d2(cur,w2):.4f}"
                    )
                pw.pop(0)
                start_trim += 1

            # Detour rule (re-evaluate after possible pop)
            if len(pw) >= 2:
                w1, w2 = pw[0], pw[1]
                d_direct = d(cur, w2)
                d_detour = d(cur, w1) + d(w1, w2)
                if d_direct + tol_m < d_detour:
                    if log:
                        self.get_logger().info(
                            "[prune] start detour: drop w1 (detour longer than direct) | "
                            f"cur={cur} w1={w1} w2={w2} | "
                            f"d(cur,w2)={d_direct:.4f} d(cur,w1)+d(w1,w2)={d_detour:.4f}"
                        )
                    pw.pop(0)
                    start_trim += 1

            # Progress rule (re-evaluate current first waypoint after possible pop)
            if len(pw) >= 1:
                w1 = pw[0]
                if d2(w1, goal) > d2(cur, goal) + tol2:
                    if log:
                        self.get_logger().info(
                            "[prune] start progress: drop w1 (behind) | "
                            f"cur={cur} goal={goal} w1={w1} | "
                            f"d2(w1,goal)={d2(w1,goal):.4f} d2(cur,goal)={d2(cur,goal):.4f}"
                        )
                    pw.pop(0)
                    start_trim += 1

        # ---------- END pruning ----------
        if len(pw) >= 2:
            wn_1, wn = pw[-2], pw[-1]

            # Neighbor rule
            if d2(goal, wn_1) + tol2 < d2(goal, wn):
                if log:
                    self.get_logger().info(
                        "[prune] end neighbor: drop wn | "
                        f"cur={cur} goal={goal} wn_1={wn_1} wn={wn} | "
                        f"d2(goal,wn_1)={d2(goal,wn_1):.4f} d2(goal,wn)={d2(goal,wn):.4f}"
                    )
                pw.pop(-1)
                end_trim += 1

            # Detour rule (re-evaluate after possible pop)
            if len(pw) >= 2:
                wn_1, wn = pw[-2], pw[-1]
                d_direct = d(wn_1, goal)
                d_detour = d(wn_1, wn) + d(wn, goal)
                if d_direct + tol_m < d_detour:
                    if log:
                        self.get_logger().info(
                            "[prune] end detour: drop wn (detour longer than direct) | "
                            f"goal={goal} wn_1={wn_1} wn={wn} | "
                            f"d(wn_1,goal)={d_direct:.4f} d(wn_1,wn)+d(wn,goal)={d_detour:.4f}"
                        )
                    pw.pop(-1)
                    end_trim += 1

            # Progress rule (re-evaluate current last waypoint after possible pop)
            if len(pw) >= 1:
                wn = pw[-1]
                if d2(wn, cur) > d2(goal, cur) + tol2:
                    if log:
                        self.get_logger().info(
                            "[prune] end progress: drop wn (beyond) | "
                            f"cur={cur} goal={goal} wn={wn} | "
                            f"d2(wn,cur)={d2(wn,cur):.4f} d2(goal,cur)={d2(goal,cur):.4f}"
                        )
                    pw.pop(-1)
                    end_trim += 1

        # ---------- 1-point collapse to empty ----------
        if len(pw) == 1:
            w = pw[0]
            if (d2(cur, w) + tol2 > d2(cur, goal)) and (d2(goal, w) + tol2 > d2(cur, goal)):
            # if (d2(cur, goal) + tol2 < d2(cur, w)) or (d2(cur, goal) + tol2 < d2(goal, w)):
                if log:
                    self.get_logger().info(
                        "[prune] single-point collapse: drop w -> empty | "
                        f"cur={cur} goal={goal} w={w} | "
                        f"d2(cur,goal)={d2(cur,goal):.4f} d2(cur,w)={d2(cur,w):.4f} d2(goal,w)={d2(goal,w):.4f}"
                    )
                pw.pop(0)
                start_trim += 1

        after = len(pw)
        if log:
            self.get_logger().info(f"[prune] result: {before} -> {after} waypoints")
        if return_trim:
            return pw, start_trim, end_trim
        return pw

    @staticmethod
    def _neighbor_offsets_for_layers(num_layers):
        if num_layers <= 0:
            return ((0, 0, 0),)

        if num_layers == 1:
            return (
                (0, 0, 0),
                (1, 0, 0), (-1, 0, 0),
                (0, 1, 0), (0, -1, 0),
                (0, 0, 1), (0, 0, -1),
            )

        return tuple(
            (di, dj, dk)
            for di in range(-num_layers, num_layers + 1)
            for dj in range(-num_layers, num_layers + 1)
            for dk in range(-num_layers, num_layers + 1)
        )

    def _log_replanning_limit_hit(self, limit_name, current_value, limit_value, context):
        self.get_logger().warn(
            f"Replanning stop trigger reached: {limit_name}={current_value} (limit={limit_value}). {context}"
        )

    def _maybe_log_wrist3_replanning_state(
        self,
        label,
        start_grid,
        goal_grid,
        start_world,
        goal_world,
        **stats,
    ):
        if self.astar_collision_mode != 'wrist3' or not self.log_detailed_planning_diagnostics:
            return

        stat_parts = [f"{key}={value}" for key, value in stats.items()]
        self.get_logger().warn(
            f"Wrist3 replanning {label}: "
            f"gap {start_grid} ({tuple(round(v, 3) for v in start_world)}) -> "
            f"{goal_grid} ({tuple(round(v, 3) for v in goal_world)}); "
            + ", ".join(stat_parts)
        )
    
    def _mark_cell_and_neighbors_as_obstacles(self, occupancy_grid, i, j, k, num_layers=1):
        """
        Mark a grid cell and optionally its neighbors as obstacles.
        This creates a safety margin around problematic grid cells during replanning.
        
        Args:
            occupancy_grid: The 3D occupancy grid to modify
            i, j, k: Grid indices of the cell to mark
            num_layers: Number of neighbor layers to mark:
                       0 = mark only the cell itself (1 cell)
                       1 = mark only the 6 axis-aligned neighbors + center (7 cells)
                       2 = mark cell + 2 layers of neighbors (5x5x5 cube = 125 cells)
                       etc.
        
        Returns:
            Number of newly blocked cells (including the center cell)
        """
        marked_count = 0

        for di, dj, dk in self._neighbor_offsets_for_layers(num_layers):
            ni, nj, nk = i + di, j + dj, k + dk
            if (
                0 <= ni < occupancy_grid.shape[0]
                and 0 <= nj < occupancy_grid.shape[1]
                and 0 <= nk < occupancy_grid.shape[2]
            ):
                if occupancy_grid[ni, nj, nk] != 1:
                    occupancy_grid[ni, nj, nk] = 1
                    marked_count += 1
        
        return marked_count
    
    def _segment_path_by_collision(self, waypoint_results):
        """
        Segment path into collision-free groups based on waypoint collision check results.
        
        Args:
            waypoint_results: List of tuples (waypoint_idx, is_collision_free, joint_solution, grid_idx)
        
        Returns:
            List of segments, where each segment is a dict:
            {
                'start_idx': first waypoint index in segment,
                'end_idx': last waypoint index in segment (inclusive),
                'waypoints': list of waypoint indices,
                'is_collision_free': bool,
                'joint_solutions': list of joint configs (None for collision segments)
            }
        """
        if not waypoint_results:
            return []
        
        segments = []
        current_segment = None
        
        for wp_idx, is_free, joint_sol, grid_idx in waypoint_results:
            if current_segment is None:
                # Start new segment
                current_segment = {
                    'start_idx': wp_idx,
                    'end_idx': wp_idx,
                    'waypoints': [wp_idx],
                    'is_collision_free': is_free,
                    'joint_solutions': [joint_sol] if is_free else [],
                    'grid_indices': [grid_idx] if grid_idx is not None else []
                }
            elif current_segment['is_collision_free'] == is_free:
                # Continue current segment
                current_segment['end_idx'] = wp_idx
                current_segment['waypoints'].append(wp_idx)
                if is_free and joint_sol is not None:
                    current_segment['joint_solutions'].append(joint_sol)
                if grid_idx is not None:
                    current_segment['grid_indices'].append(grid_idx)
            else:
                # Collision status changed - save current segment and start new one
                segments.append(current_segment)
                current_segment = {
                    'start_idx': wp_idx,
                    'end_idx': wp_idx,
                    'waypoints': [wp_idx],
                    'is_collision_free': is_free,
                    'joint_solutions': [joint_sol] if is_free else [],
                    'grid_indices': [grid_idx] if grid_idx is not None else []
                }
        
        # Add final segment
        if current_segment is not None:
            segments.append(current_segment)
        
        return segments
    
    def _identify_connected_segments(self, segments, start_idx, goal_idx):
        """
        Identify which collision-free segments are connected to start and goal.
        
        Returns:
            (start_connected_segment, goal_connected_segment, gap_info)
            where gap_info = {'needs_replan': bool, 'from_idx': int, 'to_idx': int}
        """
        if not segments:
            return None, None, {'needs_replan': True, 'from_idx': start_idx, 'to_idx': goal_idx}
        
        # Find collision-free segment containing or adjacent to start
        start_segment = None
        for seg in segments:
            if seg['is_collision_free'] and seg['start_idx'] == 0:
                start_segment = seg
                break
        
        # Find collision-free segment containing or adjacent to goal
        goal_segment = None
        for seg in reversed(segments):
            if seg['is_collision_free']:
                # Check if this segment extends to the goal
                goal_segment = seg
                break
        
        # Determine if there's a gap that needs replanning
        if start_segment is None and goal_segment is None:
            # No collision-free segments - need to replan entire path
            gap_info = {'needs_replan': True, 'from_idx': start_idx, 'to_idx': goal_idx}
        elif start_segment is not None and goal_segment is not None:
            if start_segment == goal_segment:
                # Entire path is one collision-free segment!
                gap_info = {'needs_replan': False, 'from_idx': None, 'to_idx': None}
            else:
                # Gap between start and goal segments
                gap_info = {
                    'needs_replan': True,
                    'from_idx': start_segment['end_idx'],
                    'to_idx': goal_segment['start_idx']
                }
        elif start_segment is not None:
            # Have start segment but no goal segment
            gap_info = {
                'needs_replan': True,
                'from_idx': start_segment['end_idx'],
                'to_idx': goal_idx
            }
        else:
            # Have goal segment but no start segment
            gap_info = {
                'needs_replan': True,
                'from_idx': start_idx,
                'to_idx': goal_segment['start_idx']
            }
        
        return start_segment, goal_segment, gap_info
    
    def _identify_obstacle(self, position, occupancy_grid, grid_idx):
        """
        Identify which specific obstacle a position is inside.
        Returns a descriptive string naming the obstacle.
        """
        self.get_logger().info(f"🔍 _identify_obstacle: position={position}, grid_idx={grid_idx}")
        x, y, z = position
        i, j, k = int(grid_idx[0]), int(grid_idx[1]), int(grid_idx[2])
        
        obstacles = []
        
        # Check cylinder obstacle
        self.get_logger().info(f"  Checking cylinder...")
        dist_xy = np.sqrt((x - self.cyl_center_xy[0])**2 + (y - self.cyl_center_xy[1])**2)
        if dist_xy <= self.cyl_radius and self.z_min <= z <= self.z_max:
            obstacles.append(f"Cylinder (center={self.cyl_center_xy}, radius={self.cyl_radius}m, z=[{self.z_min}, {self.z_max}])")
            self.get_logger().info(f"  ✓ Inside cylinder!")
        
        # Check base footprint
        self.get_logger().info(f"  Checking base footprint...")
        if self.base_z_min <= z <= self.base_z_max:
            # Simple point-in-polygon check
            polygon = self.base_footprint_xy
            n = len(polygon)
            inside = False
            p1x, p1y = polygon[0]
            for i_poly in range(n + 1):
                p2x, p2y = polygon[i_poly % n]
                if y > min(p1y, p2y):
                    if y <= max(p1y, p2y):
                        if x <= max(p1x, p2x):
                            if p1y != p2y:
                                xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                            if p1x == p2x or x <= xinters:
                                inside = not inside
                p1x, p1y = p2x, p2y
            
            if inside:
                obstacles.append(f"Mobile base footprint (polygon with {len(polygon)} vertices, z=[{self.base_z_min}, {self.base_z_max}])")
                self.get_logger().info(f"  ✓ Inside base footprint!")
        
        self.get_logger().info(f"  Checking occupancy_grid at [{i}, {j}, {k}]...")
        if occupancy_grid[i, j, k] == 1:
            if not obstacles:
                obstacles.append("Unknown obstacle (present in occupancy grid)")
                self.get_logger().info(f"  ✓ In occupancy grid!")
        else:
            # Only in dilated grid, not in original
            if not obstacles:
                obstacles.append("Dilated safety margin around obstacle")
                self.get_logger().info(f"  ✓ Only in dilated margin")
        
        self.get_logger().info(f"  Final obstacles list: {obstacles}")
        
        if obstacles:
            return " AND ".join(obstacles)
        else:
            return "Unknown (possibly dilation margin)"

    def _find_replanning_overlay_contributors(self, grid_idx, collision_waypoints_to_avoid):
        target_idx = tuple(int(v) for v in grid_idx)
        contributors = []

        for source_grid_idx, num_layers in collision_waypoints_to_avoid.items():
            source_idx = tuple(int(v) for v in source_grid_idx)
            offset = (
                target_idx[0] - source_idx[0],
                target_idx[1] - source_idx[1],
                target_idx[2] - source_idx[2],
            )
            if offset in self._neighbor_offsets_for_layers(int(num_layers)):
                contributors.append({
                    'grid_idx': source_idx,
                    'num_layers': int(num_layers),
                    'offset': offset,
                })

        return contributors

    def _describe_grid_blockage(
        self,
        position,
        grid_idx,
        occupancy_grid,
        occupancy_grid_dilated_base,
        occupancy_grid_dilated,
        collision_waypoints_to_avoid,
    ):
        i, j, k = (int(grid_idx[0]), int(grid_idx[1]), int(grid_idx[2]))

        if occupancy_grid[i, j, k] == 1:
            obstacle_name = self._identify_obstacle(position, occupancy_grid, grid_idx)
            return {
                'headline': 'is INSIDE STATIC ENVIRONMENT OBSTACLE',
                'reason': obstacle_name,
                'source': 'static environment occupancy grid',
            }

        if occupancy_grid_dilated_base[i, j, k] == 1:
            return {
                'headline': 'is INSIDE STATIC DILATED SAFETY MARGIN',
                'reason': 'Static dilated environment margin',
                'source': 'static occupancy-grid dilation',
            }

        if occupancy_grid_dilated[i, j, k] == 1:
            overlay_contributors = self._find_replanning_overlay_contributors(
                grid_idx,
                collision_waypoints_to_avoid,
            )
            if overlay_contributors:
                contributor_preview = ', '.join(
                    f"{item['grid_idx']} (layers={item['num_layers']}, offset={item['offset']})"
                    for item in overlay_contributors[:3]
                )
                if any(item['offset'] == (0, 0, 0) for item in overlay_contributors):
                    overlay_source = (
                        'replanning exclusion overlay; the blocked cell is itself a tracked colliding waypoint '
                        f"({contributor_preview})"
                    )
                else:
                    overlay_source = (
                        'replanning exclusion overlay; the blocked cell lies inside the neighbor exclusion region '
                        f"of tracked collision waypoint(s) ({contributor_preview})"
                    )
            else:
                overlay_source = 'replanning exclusion overlay from previously colliding waypoint(s)'

            return {
                'headline': 'is BLOCKED IN CURRENT REPLANNING GRID',
                'reason': 'Replanning exclusion around previously colliding waypoint(s)',
                'source': overlay_source,
            }

        return {
            'headline': 'is BLOCKED FOR AN UNKNOWN GRID REASON',
            'reason': 'Unknown grid blockage source',
            'source': 'current planning grid state could not be classified',
        }

    def _format_endpoint_block_message(
        self,
        endpoint_scope,
        frame_label,
        position,
        paired_frame_label,
        paired_position,
        grid_idx,
        occupancy_grid,
        occupancy_grid_dilated_base,
        occupancy_grid_dilated,
        collision_waypoints_to_avoid,
        replanning_attempt,
        current_planning_start_grid=None,
        current_planning_goal_grid=None,
    ):
        blockage = self._describe_grid_blockage(
            position,
            grid_idx,
            occupancy_grid,
            occupancy_grid_dilated_base,
            occupancy_grid_dilated,
            collision_waypoints_to_avoid,
        )

        lines = [
            f"❌ {endpoint_scope} {frame_label} position {blockage['headline']}: {blockage['reason']}",
            f"  Scope: {endpoint_scope}",
            f"  Position ({frame_label}): {[round(x, 3) for x in position]}",
            f"  Position ({paired_frame_label}): {[round(x, 3) for x in paired_position]}",
            f"  Grid index ({frame_label}): {grid_idx}",
            f"  Blocking source: {blockage['source']}",
        ]

        if replanning_attempt > 0:
            lines.append(
                f"  Current replanning gap (wrist_3 grid): {current_planning_start_grid} -> {current_planning_goal_grid}"
            )
            lines.append(
                '  Note: this check is being evaluated inside the replanning loop against the current replanning grid, not only against the original static environment grid.'
            )

        lines.append('  → Cannot continue planning from this blocked endpoint check.')
        return '\n'.join(lines)
        
    def emergency_callback(self, msg):
        with self._state_lock:
            emergency_activated = msg.data and not self.emergency_stop
            emergency_cleared = (not msg.data) and self.emergency_stop
            self.emergency_stop = msg.data
            if self.emergency_stop:
                self.execution_complete = True
                if emergency_activated:
                    self._plan_generation += 1

        if emergency_activated:
            self.get_logger().warn("Emergency stop activated. Active planning work will be cancelled.")
        elif emergency_cleared:
            self.get_logger().info("Emergency stop cleared. Planner can resume queued work.")

    def reset_callback(self, msg: Bool):
        """
        Reset planner state to initialization values when requested.
        This allows unstucking the planner when goals are stuck due to inactive controllers.
        """
        if msg.data:
            with self._state_lock:
                queued_goals = len(self.goal_queue)
                planning_was_active = self._planning_active
                self.execution_complete = True
                self.goal_queue = []
                self.emergency_stop = False
                self._plan_generation += 1
            
            # Clear visualization markers
            self.clear_goal_and_path_markers()
            
            self.get_logger().warn(
                "Planner state RESET requested and completed. "
                f"Cleared {queued_goals} queued goals. "
                f"{'Cancelled active planning. ' if planning_was_active else ''}"
                "Ready to accept new goals."
            )
            
            # Publish that any previous goal has failed (since we're resetting)
            self.publish_planner_goal_failed(True)

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

        with self._state_lock:
            self.joint_indices = joint_indices
            self.current_joint_state = msg
        

    def end_effector_pose_callback(self, msg):
        with self._state_lock:
            self.end_effector_pose = msg

    def execution_status_callback(self, msg: Bool):
        with self._state_lock:
            self.execution_complete = msg.data
            queued_goals = len(self.goal_queue)

        if msg.data:
            self.get_logger().info("Goal execution complete. Ready for new goal.")
            if queued_goals:
                self.get_logger().info(f"{queued_goals} queued goal(s) waiting for the planner worker.")

    def publish_planner_goal_failed(self, failed: bool):
        msg = Bool()
        msg.data = bool(failed)
        self.planner_goal_failed_pub.publish(msg)

    def _fail_current_goal(self, message: str):
        self.get_logger().error(message)
        self._log_plan_metrics('failed')
        with self._state_lock:
            self.execution_complete = True
        self.publish_planner_goal_failed(True)

    def _reset_plan_metrics(self):
        self._plan_started_at = None
        self._plan_metrics_logged = False
        self._collision_check_request_count = 0
        self._collision_check_batch_request_count = 0
        self._collision_check_call_count = 0
        self._collision_check_total_sec = 0.0
        self._collision_check_free_count = 0
        self._collision_check_collision_count = 0
        self._collision_check_failed_count = 0
        self._collision_check_cache = {}
        self._collision_check_cache_hits = 0
        self._collision_check_cache_misses = 0
        self._astar_search_count = 0
        self._astar_dual_frame_cache_hits = 0
        self._astar_dual_frame_cache_misses = 0
        self._astar_dual_frame_cache_entries = 0
        self._astar_iteration_total = 0

    def _start_plan_metrics(self):
        self._reset_plan_metrics()
        self._plan_started_at = time.monotonic()

    def _record_astar_search_metrics(self, path_stats):
        if not path_stats:
            return

        self._astar_search_count += 1
        self._astar_iteration_total += int(path_stats.get('iterations', 0))
        self._astar_dual_frame_cache_hits += int(path_stats.get('dual_frame_cache_hits', 0))
        self._astar_dual_frame_cache_misses += int(path_stats.get('dual_frame_cache_misses', 0))
        self._astar_dual_frame_cache_entries += int(path_stats.get('dual_frame_cache_entries', 0))

    def _make_collision_cache_key(self, joint_values):
        canonical_joint_values = np.arctan2(np.sin(joint_values), np.cos(joint_values))
        rounded_joint_values = np.round(canonical_joint_values, self.collision_cache_round_digits)
        return tuple(float(value) for value in rounded_joint_values)

    def _make_collision_request_joint_state(self, joint_values):
        joint_state = JointState()
        joint_state.name = [name.replace(self.joint_prefix, '') for name in self.expected_joint_names]
        joint_state.position = joint_values.tolist()
        return joint_state

    def _record_collision_request_metric(self, request_count=1, batched=False):
        self._collision_check_request_count += int(request_count)
        if batched:
            self._collision_check_batch_request_count += int(request_count)

    def _record_collision_check_metric(self, status: CollisionCheckStatus, elapsed_sec: float):
        self._collision_check_call_count += 1
        self._collision_check_total_sec += float(elapsed_sec)
        if status == CollisionCheckStatus.FREE:
            self._collision_check_free_count += 1
        elif status == CollisionCheckStatus.COLLISION:
            self._collision_check_collision_count += 1
        else:
            self._collision_check_failed_count += 1

        if (
            self.log_performance_metrics
            and self.collision_metrics_log_interval > 0
            and self._collision_check_call_count % self.collision_metrics_log_interval == 0
        ):
            avg_ms = (self._collision_check_total_sec / self._collision_check_call_count) * 1000.0
            self.get_logger().warn(
                f"Planner collision metrics: checks={self._collision_check_call_count} "
                f"requests={self._collision_check_request_count} "
                f"batch_requests={self._collision_check_batch_request_count} "
                f"avg_ms={avg_ms:.2f} free={self._collision_check_free_count} "
                f"collision={self._collision_check_collision_count} "
                f"failed={self._collision_check_failed_count} "
                f"cache_hits={self._collision_check_cache_hits} "
                f"cache_entries={len(self._collision_check_cache)}"
            )

    def _log_plan_metrics(self, outcome: str):
        if (not self.log_performance_metrics) or self._plan_started_at is None or self._plan_metrics_logged:
            return

        total_sec = time.monotonic() - self._plan_started_at
        avg_collision_ms = 0.0
        if self._collision_check_call_count > 0:
            avg_collision_ms = (self._collision_check_total_sec / self._collision_check_call_count) * 1000.0

        self.get_logger().warn(
            f"Planner metrics [{outcome}]: total_sec={total_sec:.2f} "
            f"astar_searches={self._astar_search_count} "
            f"astar_iters={self._astar_iteration_total} "
            f"astar_cache_hits={self._astar_dual_frame_cache_hits} "
            f"astar_cache_misses={self._astar_dual_frame_cache_misses} "
            f"astar_cache_entries={self._astar_dual_frame_cache_entries} "
            f"collision_checks={self._collision_check_call_count} "
            f"collision_requests={self._collision_check_request_count} "
            f"batch_requests={self._collision_check_batch_request_count} "
            f"avg_collision_ms={avg_collision_ms:.2f} free={self._collision_check_free_count} "
            f"collision={self._collision_check_collision_count} "
            f"failed={self._collision_check_failed_count} "
            f"cache_hits={self._collision_check_cache_hits} "
            f"cache_entries={len(self._collision_check_cache)}"
        )
        self._plan_metrics_logged = True

    def _snapshot_planning_inputs(self):
        with self._state_lock:
            joint_state = copy.deepcopy(self.current_joint_state)
            end_effector_pose = copy.deepcopy(self.end_effector_pose)
            joint_indices = None if self.joint_indices is None else list(self.joint_indices)
        return joint_state, end_effector_pose, joint_indices

    def _should_abort_plan(self, plan_generation):
        if plan_generation is None:
            return False
        with self._state_lock:
            return self.emergency_stop or self._plan_generation != plan_generation

    def _abort_if_requested(self, plan_generation, context):
        if self._should_abort_plan(plan_generation):
            self.get_logger().warn(f"Aborting planning: {context}")
            return True
        return False

    def _process_goal_queue(self):
        with self._state_lock:
            if (
                self.emergency_stop
                or self._planning_active
                or not self.execution_complete
                or not self.goal_queue
            ):
                return

            next_goal = self.goal_queue.pop(0)
            self._planning_active = True
            self.execution_complete = False
            plan_generation = self._plan_generation

        self.publish_planner_goal_failed(False)
        if self._should_publish_planner_markers():
            time.sleep(5.0)  # DEBUG: pause so RViz markers are visible before being cleared
            self.clear_goal_and_path_markers()
        pose_data = (
            next_goal.pose.position.x,
            next_goal.pose.position.y,
            next_goal.pose.position.z,
            next_goal.pose.orientation.x,
            next_goal.pose.orientation.y,
            next_goal.pose.orientation.z,
            next_goal.pose.orientation.w
        )
        if self._should_publish_planner_markers():
            self.publish_goal_marker(pose_data)
            self.publish_reachability_boundary_marker()
        self._planning_thread = threading.Thread(
            target=self._plan_goal_worker,
            args=(copy.deepcopy(next_goal), plan_generation),
            daemon=True,
        )
        self._planning_thread.start()

    def _plan_goal_worker(self, msg: PoseStamped, plan_generation: int):
        try:
            if self._abort_if_requested(plan_generation, "goal was cancelled before planning began"):
                return
            self.plan_and_send_trajectory(msg, plan_generation=plan_generation)
        except Exception as exc:
            import traceback
            tb_str = ''.join(traceback.format_exception(type(exc), exc, exc.__traceback__))
            self.get_logger().error(f"Exception traceback:\n{tb_str}")
            self._fail_current_goal(f"Unhandled exception while planning goal: {exc}")
        finally:
            with self._state_lock:
                self._planning_active = False

    def goal_callback(self, msg: PoseStamped):
        with self._state_lock:
            if self.emergency_stop:
                emergency_active = True
                queued_goals = len(self.goal_queue)
            else:
                emergency_active = False
                self.goal_queue.append(copy.deepcopy(msg))
                queued_goals = len(self.goal_queue)
                planning_busy = self._planning_active or not self.execution_complete

        if emergency_active:
            self.get_logger().warn("Emergency stop is active, aborting trajectory planning. Ignoring goal.")
            return

        if planning_busy or queued_goals > 1:
            self.get_logger().warn("Previous trajectory not finished. Goal queued.")
            return

        self.get_logger().info("Goal queued for planning worker.")

    def _sort_ik_solutions_by_distance(self, ik_solutions, q_current):
        """
        Sort IK solutions by their distance from the current joint configuration.
        
        Args:
            ik_solutions: (8, 6) array of IK solutions
            q_current: (6,) array of current joint values
            
        Returns:
            sorted_indices: Array of indices sorting solutions from closest to furthest
            distances: Array of distances corresponding to each solution
        """
        # Filter out invalid solutions (rows with NaN)
        valid_mask = ~np.any(np.isnan(ik_solutions), axis=1)
        # Check if service became available (may have started after initial checks)
        if not self._collision_service_available and self.collision_check_client.service_is_ready():
            self._collision_service_available = True
            self.get_logger().info("✅ Collision checking service now available (late connection)")
        
        valid_indices = np.where(valid_mask)[0]
        
        if len(valid_indices) == 0:
            return np.array([]), np.array([])
        
        # Calculate weighted distance for each valid solution
        # Use angular distance in joint space
        weights = np.ones(6)
        distances = []
        
        for idx in valid_indices:
            # Angular distance (handles wraparound at ±π)
            delta = np.arctan2(np.sin(q_current - ik_solutions[idx]), 
                              np.cos(q_current - ik_solutions[idx]))
            dist = np.sqrt(np.sum(weights * delta**2))
            distances.append(dist)
        
        distances = np.array(distances)
        sorted_order = np.argsort(distances)
        sorted_indices = valid_indices[sorted_order]
        sorted_distances = distances[sorted_order]
        
        return sorted_indices, sorted_distances

    @staticmethod
    def _format_ik_target_label(waypoint_idx):
        if waypoint_idx == "GOAL":
            return "Final goal pose"
        if waypoint_idx is None:
            return "IK target"
        return f"Intermediate path waypoint {waypoint_idx}"

    def _get_ranked_ik_candidates(self, ik_solutions, q_current, waypoint_idx=None):
        """
        Return IK candidates ordered by proximity to the reference configuration.

        The existing `max_ik_solutions_to_test` parameter limits how many nearby
        candidates are considered for both direct selection and collision fallback.
        """
        ik_solutions_array = np.asarray(ik_solutions, dtype=float)
        target_label = self._format_ik_target_label(waypoint_idx)
        if ik_solutions_array.ndim == 1:
            if np.any(np.isnan(ik_solutions_array)):
                if waypoint_idx is not None and self.log_detailed_planning_diagnostics:
                    self.get_logger().error(f"{target_label}: no valid IK solutions found")
                return []

            delta = np.arctan2(np.sin(q_current - ik_solutions_array), np.cos(q_current - ik_solutions_array))
            candidate_dist = float(np.sqrt(np.sum(delta ** 2)))
            return [(0, ik_solutions_array.copy(), candidate_dist)]

        sorted_indices, sorted_distances = self._sort_ik_solutions_by_distance(ik_solutions, q_current)

        if len(sorted_indices) == 0:
            if waypoint_idx is not None and self.log_detailed_planning_diagnostics:
                self.get_logger().error(f"{target_label}: no valid IK solutions found")
            return []

        if self.max_ik_solutions_to_test > 0:
            original_count = len(sorted_indices)
            sorted_indices = sorted_indices[:self.max_ik_solutions_to_test]
            sorted_distances = sorted_distances[:self.max_ik_solutions_to_test]
            if waypoint_idx == 0:
                self._log_hot_loop_info(lambda: (
                    f"Testing only {len(sorted_indices)}/{original_count} closest IK solutions "
                    f"(max_ik_solutions_to_test={self.max_ik_solutions_to_test})"
                ))

        ranked_candidates = []
        for candidate_idx, candidate_dist in zip(sorted_indices, sorted_distances):
            ranked_candidates.append(
                (int(candidate_idx), np.asarray(ik_solutions[candidate_idx], dtype=float).copy(), float(candidate_dist))
            )
        return ranked_candidates

    def _select_collision_free_ranked_candidate(
        self,
        ranked_candidates,
        waypoint_idx,
        timeout_sec=2.0,
        cancel_check=None,
    ):
        """
        Try ranked IK candidates in order until one validates collision-free.
        """
        if not ranked_candidates:
            return None, IKSelectionOutcome.NO_VALID_SOLUTIONS

        target_label = self._format_ik_target_label(waypoint_idx)

        if cancel_check is not None and cancel_check():
            return None, IKSelectionOutcome.CHECK_FAILED

        collided_candidate_indices = []
        for candidate_rank, (candidate_idx, candidate_solution, candidate_dist) in enumerate(ranked_candidates, start=1):
            collision_status = self._check_collision_sync(
                candidate_solution,
                publish_visualization=self.visualize_collision_checks,
                timeout_sec=timeout_sec,
                cancel_check=cancel_check,
            )

            if collision_status == CollisionCheckStatus.CHECK_FAILED:
                if self.log_detailed_planning_diagnostics:
                    self.get_logger().warn(
                        f"{target_label}: IK candidate {candidate_idx} could not be collision-validated "
                        f"(rank {candidate_rank}, dist: {candidate_dist:.3f})"
                    )
                return None, IKSelectionOutcome.CHECK_FAILED

            if collision_status == CollisionCheckStatus.FREE:
                if candidate_rank == 1 and self.log_detailed_planning_diagnostics:
                    self._log_hot_loop_info(lambda: (
                        f"{target_label}: closest IK solution {candidate_idx} is collision-free "
                        f"(dist: {candidate_dist:.3f})"
                    ))
                elif self.log_detailed_planning_diagnostics:
                    self.get_logger().warn(
                        f"{target_label}: recovered with fallback IK solution {candidate_idx} "
                        f"(rank {candidate_rank}, dist: {candidate_dist:.3f}) after collisions in {collided_candidate_indices}"
                    )
                return candidate_solution, IKSelectionOutcome.SUCCESS

            collided_candidate_indices.append(candidate_idx)

        if self.log_detailed_planning_diagnostics:
            self.get_logger().warn(
                f"{target_label}: all tested IK solutions are in collision {collided_candidate_indices}"
            )
        return None, IKSelectionOutcome.ALL_IN_COLLISION

    def _check_collision_sync(
        self,
        joint_values,
        publish_visualization=False,
        timeout_sec=2.0,
        cancel_check=None,
        max_retries=2,
    ):
        """
        Check collision using a service request while allowing the executor
        thread to remain free to receive the response.
        
        Args:
            joint_values: (6,) array of joint values
            publish_visualization: Whether to publish collision visualization markers
            timeout_sec: Service call timeout in seconds
            cancel_check: Optional callable that returns True when the current
                planning job should abort
            max_retries: Maximum number of retry attempts if service fails (default: 2)
            
        Returns:
            CollisionCheckStatus indicating whether the tested state is free,
            in collision, or could not be validated.
        """
        started_at = time.monotonic()

        def finish(status):
            self._record_collision_request_metric(request_count=1, batched=False)
            self._record_collision_check_metric(status, time.monotonic() - started_at)
            return status

        cache_key = None
        if self.cache_collision_checks and not publish_visualization:
            cache_key = self._make_collision_cache_key(np.asarray(joint_values, dtype=float))
            cached_status = self._collision_check_cache.get(cache_key)
            if cached_status is not None:
                self._collision_check_cache_hits += 1
                return cached_status
            self._collision_check_cache_misses += 1

        if not self.collision_check_client.service_is_ready():
            wait_timeout = max(0.1, min(timeout_sec, 0.5))
            if not self.collision_check_client.wait_for_service(timeout_sec=wait_timeout):
                self.get_logger().warn(
                    f"Collision check service '/collision/check_collision_pose' is not ready after waiting {wait_timeout:.2f}s."
                )
                return finish(CollisionCheckStatus.CHECK_FAILED)
        
        request = CheckCollisionPose.Request()
        request.joint_state = self._make_collision_request_joint_state(joint_values)
        request.publish_visualization = publish_visualization
        
        # Retry loop for handling transient service failures
        for attempt in range(max_retries + 1):
            try:
                # Create event to signal when response is received
                response_event = threading.Event()
                result_container = {'response': None, 'error': None}
                
                def response_callback(future):
                    try:
                        result_container['response'] = future.result()
                    except Exception as e:
                        result_container['error'] = e
                    finally:
                        response_event.set()
                
                # Call service asynchronously with callback
                future = self.collision_check_client.call_async(request)
                future.add_done_callback(response_callback)
                
                deadline = time.monotonic() + timeout_sec
                while True:
                    if cancel_check is not None and cancel_check():
                        future.cancel()
                        self.get_logger().warn("Collision validation cancelled because planning was aborted.")
                        return finish(CollisionCheckStatus.CHECK_FAILED)

                    remaining = deadline - time.monotonic()
                    if remaining <= 0.0:
                        if attempt < max_retries:
                            self.get_logger().warn(
                                f"Collision check timed out after {timeout_sec}s (attempt {attempt + 1}/{max_retries + 1}), retrying..."
                            )
                            break  # Break inner while loop to retry
                        else:
                            self.get_logger().warn(
                                f"Collision check timed out after {timeout_sec}s (final attempt {attempt + 1}/{max_retries + 1})"
                            )
                            return finish(CollisionCheckStatus.CHECK_FAILED)

                    if response_event.wait(timeout=min(0.05, remaining)):
                        if result_container['error']:
                            if attempt < max_retries:
                                self.get_logger().warn(
                                    f"Collision check service error (attempt {attempt + 1}/{max_retries + 1}): {result_container['error']}, retrying..."
                                )
                                break  # Break inner while loop to retry
                            else:
                                self.get_logger().warn(
                                    f"Collision check service error (final attempt {attempt + 1}/{max_retries + 1}): {result_container['error']}"
                                )
                                return finish(CollisionCheckStatus.CHECK_FAILED)
                        
                        # Success! Return the result
                        status = (
                            CollisionCheckStatus.COLLISION
                            if result_container['response'].in_collision
                            else CollisionCheckStatus.FREE
                        )
                        if cache_key is not None:
                            self._collision_check_cache[cache_key] = status
                        return finish(status)
                
            except Exception as e:
                if attempt < max_retries:
                    self.get_logger().warn(
                        f"Collision check service call failed (attempt {attempt + 1}/{max_retries + 1}): {e}, retrying..."
                    )
                else:
                    self.get_logger().warn(
                        f"Collision check service call failed (final attempt {attempt + 1}/{max_retries + 1}): {e}"
                    )
                    return finish(CollisionCheckStatus.CHECK_FAILED)
        
        # Should not reach here, but for safety
        return finish(CollisionCheckStatus.CHECK_FAILED)

    def _check_collision_batch_sync(
        self,
        joint_values_batch,
        publish_visualization_index=None,
        timeout_sec=2.0,
        cancel_check=None,
        max_retries=2,
    ):
        if not joint_values_batch:
            return []

        statuses = [None] * len(joint_values_batch)
        uncached_entries = []

        for original_idx, joint_values in enumerate(joint_values_batch):
            joint_values_array = np.asarray(joint_values, dtype=float)
            visualize = publish_visualization_index == original_idx
            cache_key = None

            if self.cache_collision_checks and not visualize:
                cache_key = self._make_collision_cache_key(joint_values_array)
                cached_status = self._collision_check_cache.get(cache_key)
                if cached_status is not None:
                    self._collision_check_cache_hits += 1
                    statuses[original_idx] = cached_status
                    continue
                self._collision_check_cache_misses += 1

            uncached_entries.append((original_idx, joint_values_array, cache_key, visualize))

        if not uncached_entries:
            return statuses

        def fallback_to_single_checks():
            for original_idx, joint_values_array, _, visualize in uncached_entries:
                statuses[original_idx] = self._check_collision_sync(
                    joint_values_array,
                    publish_visualization=visualize,
                    timeout_sec=timeout_sec,
                    cancel_check=cancel_check,
                    max_retries=max_retries,
                )
            return statuses

        if (
            not self.enable_batch_collision_checks
            or len(uncached_entries) == 1
            or not self.collision_check_batch_client.service_is_ready()
        ):
            if len(uncached_entries) > 1 and not self.collision_check_batch_client.service_is_ready():
                wait_timeout = max(0.1, min(timeout_sec, 0.5))
                if self.collision_check_batch_client.wait_for_service(timeout_sec=wait_timeout):
                    pass
                else:
                    self.get_logger().warn(
                        f"Batch collision service '/collision/check_collision_poses' is not ready after waiting {wait_timeout:.2f}s; falling back to single checks."
                    )
            return fallback_to_single_checks()

        request_entries = [entry for entry in uncached_entries if entry[3]] + [
            entry for entry in uncached_entries if not entry[3]
        ]

        request = CheckCollisionPoses.Request()
        request.publish_visualization = any(entry[3] for entry in request_entries)
        request.joint_states = [
            self._make_collision_request_joint_state(joint_values_array)
            for _, joint_values_array, _, _ in request_entries
        ]

        for attempt in range(max_retries + 1):
            started_at = time.monotonic()
            try:
                response_event = threading.Event()
                result_container = {'response': None, 'error': None}

                def response_callback(future):
                    try:
                        result_container['response'] = future.result()
                    except Exception as e:
                        result_container['error'] = e
                    finally:
                        response_event.set()

                future = self.collision_check_batch_client.call_async(request)
                future.add_done_callback(response_callback)

                deadline = time.monotonic() + timeout_sec
                while True:
                    if cancel_check is not None and cancel_check():
                        future.cancel()
                        self.get_logger().warn("Batched collision validation cancelled because planning was aborted.")
                        return fallback_to_single_checks()

                    remaining = deadline - time.monotonic()
                    if remaining <= 0.0:
                        if attempt < max_retries:
                            self.get_logger().warn(
                                f"Batched collision check timed out after {timeout_sec}s (attempt {attempt + 1}/{max_retries + 1}), retrying..."
                            )
                            break

                        self.get_logger().warn(
                            f"Batched collision check timed out after {timeout_sec}s (final attempt {attempt + 1}/{max_retries + 1}); falling back to single checks."
                        )
                        return fallback_to_single_checks()

                    if response_event.wait(timeout=min(0.05, remaining)):
                        if result_container['error']:
                            if attempt < max_retries:
                                self.get_logger().warn(
                                    f"Batched collision check service error (attempt {attempt + 1}/{max_retries + 1}): {result_container['error']}, retrying..."
                                )
                                break

                            self.get_logger().warn(
                                f"Batched collision check service error (final attempt {attempt + 1}/{max_retries + 1}): {result_container['error']}; falling back to single checks."
                            )
                            return fallback_to_single_checks()

                        response = result_container['response']
                        if (
                            len(response.in_collision) != len(request_entries)
                            or len(response.check_succeeded) != len(request_entries)
                        ):
                            self.get_logger().warn(
                                "Batched collision check returned mismatched response sizes; falling back to single checks."
                            )
                            return fallback_to_single_checks()

                        self._record_collision_request_metric(request_count=1, batched=True)
                        per_state_elapsed = (time.monotonic() - started_at) / max(len(request_entries), 1)
                        for response_idx, (original_idx, _, cache_key, _) in enumerate(request_entries):
                            if not response.check_succeeded[response_idx]:
                                status = CollisionCheckStatus.CHECK_FAILED
                            else:
                                status = (
                                    CollisionCheckStatus.COLLISION
                                    if response.in_collision[response_idx]
                                    else CollisionCheckStatus.FREE
                                )

                            if cache_key is not None and status != CollisionCheckStatus.CHECK_FAILED:
                                self._collision_check_cache[cache_key] = status

                            statuses[original_idx] = status
                            self._record_collision_check_metric(status, per_state_elapsed)

                        return statuses

            except Exception as e:
                if attempt < max_retries:
                    self.get_logger().warn(
                        f"Batched collision check service call failed (attempt {attempt + 1}/{max_retries + 1}): {e}, retrying..."
                    )
                else:
                    self.get_logger().warn(
                        f"Batched collision check service call failed (final attempt {attempt + 1}/{max_retries + 1}): {e}; falling back to single checks."
                    )
                    return fallback_to_single_checks()

        return fallback_to_single_checks()
    def _select_best_collision_free_solution(
        self,
        ik_solutions,
        q_current,
        waypoint_idx=-1,
        test_collision=True,
        timeout_sec=2.0,
        cancel_check=None,
    ):
        """
        Select the CLOSEST IK solution and optionally validate only that solution.
        
        This matches the original behavior where the robot always picks the smoothest path
        by preferring solutions closest to the current joint state.
        
        Args:
            ik_solutions: (8, 6) array of all IK solutions
            q_current: (6,) array of current joint configuration
            waypoint_idx: Index of waypoint for logging purposes
            test_collision: Whether to test for collisions (default: True)
            timeout_sec: Timeout per collision check in seconds (default: 2.0s)
            cancel_check: Optional callable that aborts pending collision checks
            
        Returns:
            best_solution: (6,) array of closest collision-free solution, or None if none found
            IKSelectionOutcome describing why the selection succeeded or failed
        """
        ranked_candidates = self._get_ranked_ik_candidates(ik_solutions, q_current, waypoint_idx=waypoint_idx)
        target_label = self._format_ik_target_label(waypoint_idx)

        if not ranked_candidates:
            return None, IKSelectionOutcome.NO_VALID_SOLUTIONS

        single_service_ready = self.collision_check_client.service_is_ready()
        if not test_collision:
            best_idx, best_solution, best_dist = ranked_candidates[0]
            if self.log_detailed_planning_diagnostics:
                self.get_logger().info(
                    f"{target_label}: using closest IK solution {best_idx} "
                    f"(dist: {best_dist:.3f}, collision checking disabled)"
                )
            return best_solution, IKSelectionOutcome.SUCCESS

        if not single_service_ready:
            if self.log_detailed_planning_diagnostics:
                self.get_logger().error(
                    f"{target_label}: collision checking requested but the service is unavailable"
                )
            return None, IKSelectionOutcome.CHECK_FAILED

        return self._select_collision_free_ranked_candidate(
            ranked_candidates,
            waypoint_idx=waypoint_idx,
            timeout_sec=timeout_sec,
            cancel_check=cancel_check,
        )

    def _normalize_joint_solution(self, joint_values, q_reference):
        normalized = np.asarray(joint_values, dtype=float).copy()
        q_reference = np.asarray(q_reference, dtype=float)

        delta = (normalized - q_reference + np.pi) % (2 * np.pi) - np.pi
        normalized = q_reference + delta

        for joint_idx in range(len(normalized)):
            while normalized[joint_idx] > 2 * np.pi:
                normalized[joint_idx] -= 2 * np.pi
            while normalized[joint_idx] < -2 * np.pi:
                normalized[joint_idx] += 2 * np.pi

        return normalized

    def _log_hot_loop_info(self, message_factory):
        if not self.hot_loop_info_logging:
            return

        if callable(message_factory):
            self.get_logger().info(message_factory())
            return

        self.get_logger().info(message_factory)

    def _should_publish_replanning_visualization(self):
        if not self.publish_replanning_visualization:
            return False

        return (
            self.goal_marker_pub.get_subscription_count() > 0
            or self.ee_path_marker_pub.get_subscription_count() > 0
        )

    def _should_publish_planner_markers(self):
        if not self.publish_replanning_visualization:
            return False

        return (
            self.goal_marker_pub.get_subscription_count() > 0
            or self.ee_path_marker_pub.get_subscription_count() > 0
            or self.cyl_marker_pub.get_subscription_count() > 0
            or self.wall_marker_pub.get_subscription_count() > 0
            or self.reachability_marker_pub.get_subscription_count() > 0
        )

    def plan_and_send_trajectory(self, msg: PoseStamped, plan_generation=None):
        self._start_plan_metrics()
        current_joint_state, end_effector_pose, joint_indices = self._snapshot_planning_inputs()

        if self._abort_if_requested(plan_generation, "goal was cancelled before planning state was snapshotted"):
            return

        if current_joint_state is None:
            self._fail_current_goal("No current joint state received yet. Cannot calculate trajectory.")
            return

        if end_effector_pose is None:
            self._fail_current_goal("No end effector pose received yet. Cannot calculate trajectory.")
            return

        if not joint_indices:
            self._fail_current_goal("No valid arm joint-state indices available.")
            return

        if any(i >= len(current_joint_state.position) for i in joint_indices):
            self._fail_current_goal("Received inconsistent joint_states message. Waiting for a valid arm joint-state frame.")
            return
        
        # Ensure wrist_3 <-> tool0 transforms are loaded (CRITICAL - no fallback for safety)
        if self._T_wrist3_tool0 is None or self._T_tool0_wrist3 is None:
            self._fail_current_goal( f"❌ CRITICAL: Transforms not yet loaded from TF tree. Cannot plan trajectory safely. Waiting for {self.joint_prefix}wrist_3_link <-> {self.joint_prefix}tool0 transform. Ensure robot_state_publisher is running and publishing the URDF.")
            return
        
        # Goal definition (in arm_tool0 frame)
        goal_pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        goal_orn = [msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w]
        goal_orientation = R.from_quat(goal_orn)
        self.get_logger().info(f"Received goal position (tool0): {goal_pos}")
        self.get_logger().info(f"Received goal orientation (tool0): {goal_orn}")

        # Convert goal from arm_tool0 to arm_wrist_3_link (IK targets the wrist_3 frame)
        # tool0 and wrist_3 share the same orientation; wrist_3 is 15 cm behind along tool0 Z
        T_wg_tool0 = np.eye(4)
        T_wg_tool0[:3, :3] = goal_orientation.as_matrix()
        T_wg_tool0[:3, 3] = np.array(goal_pos)
        T_wg_wrist3 = T_wg_tool0 @ self._T_tool0_wrist3
        goal_pos_wrist3 = T_wg_wrist3[:3, 3].tolist()
        goal_orientation_wrist3 = R.from_matrix(T_wg_wrist3[:3, :3])
        self.get_logger().info(f"Converted goal position (wrist_3): {goal_pos_wrist3}")

        # Example start position (NEEDS TO be parameterized)
        # if self.i == 0:
        #     start_pos = [-0.25, 0.6, 0.2]
        #     start_orientation = R.from_euler('xyz', [60, 120, 150], degrees=True)
        #     self.i += 1
        # else:
        
        # Obtainin gcurrent robot pose from JointState
        start_pos = [end_effector_pose.pose.position.x, end_effector_pose.pose.position.y, end_effector_pose.pose.position.z]
        start_orn = [end_effector_pose.pose.orientation.x, end_effector_pose.pose.orientation.y, end_effector_pose.pose.orientation.z, end_effector_pose.pose.orientation.w]
        start_orientation = R.from_quat(start_orn)

        # Convert start from arm_tool0 to arm_wrist_3_link
        T_ws_tool0 = np.eye(4)
        T_ws_tool0[:3, :3] = start_orientation.as_matrix()
        T_ws_tool0[:3, 3] = np.array(start_pos)
        T_ws_wrist3 = T_ws_tool0 @ self._T_tool0_wrist3
        start_pos_wrist3 = T_ws_wrist3[:3, 3].tolist()
        start_orientation_wrist3 = R.from_matrix(T_ws_wrist3[:3, :3])
            
        start_idx, start_in_bounds = world_to_grid_checked(
            *start_pos_wrist3, self.x_vals, self.y_vals, self.z_vals
        )
        goal_idx, goal_in_bounds = world_to_grid_checked(
            *goal_pos_wrist3, self.x_vals, self.y_vals, self.z_vals
        )
        
        # Check if tool0 positions are outside workspace bounds (warn but don't fail if wrist_3 is ok)
        start_tool0_idx, start_tool0_in_bounds = world_to_grid_checked(
            *start_pos, self.x_vals, self.y_vals, self.z_vals
        )
        if not start_tool0_in_bounds:
            self.get_logger().info(
                f"ℹ️  Start tool0 extends outside workspace bounds (expected when tool extends beyond wrist_3).\n"
                f"  Start (tool0): {[round(x, 3) for x in start_pos]} (outside grid)\n"
                f"  Start (wrist_3): {[round(x, 3) for x in start_pos_wrist3]} → grid {start_idx}"
            )
        
        goal_tool0_idx, goal_tool0_in_bounds = world_to_grid_checked(
            *goal_pos, self.x_vals, self.y_vals, self.z_vals
        )
        if not goal_tool0_in_bounds:
            self.get_logger().info(
                f"ℹ️  Goal tool0 extends outside workspace bounds (expected when tool extends beyond wrist_3).\n"
                f"  Goal (tool0): {[round(x, 3) for x in goal_pos]} (outside grid)\n"
                f"  Goal (wrist_3): {[round(x, 3) for x in goal_pos_wrist3]} → grid {goal_idx}"
            )

        # Occupancy grid: all free (NEEDS TO be parameterized)
        occupancy_grid = np.zeros(self.grid_shape, dtype=np.uint8)
        # Obstacles
        # 3D meshgrid of coordinates
        X, Y, Z = np.meshgrid(self.x_vals, self.y_vals, self.z_vals, indexing='ij')  # shape: (grid_size, grid_size, num_z)

        # Define cube bounds (in world coordinates)
        cube_center = (0.2, 0.2, 1)  # (x, y, z)
        cube_size = 0.4  # length of each side

        # Compute bounds
        x0, y0, z0 = cube_center
        half = cube_size / 2

        cube_mask = (
            (X >= x0 - half) & (X <= x0 + half) &
            (Y >= y0 - half) & (Y <= y0 + half) &
            (Z >= z0 - half) & (Z <= z0 + half)
        )
        # occupancy_grid[cube_mask] = 1

        # Define sphere parameters
        # sphere_center = [(-0.3, 0.0, 0.4), (0.3, 0.5, 0.4)]
        # sphere_radius = [0.3, 0.3]
        sphere_center = [(0.0, 0.0, 0.0), (0.0, 0.0, 0.15), (0.0, 0.0, 0.3), (0.0, 0.0, 0.45), (0.0, 0.0, 0.60), (0.0, 0.0, 0.75), (0.0, 0.0, 0.90)]
        sphere_radius = [0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2]

        # Create mask for sphere
        for i in range(len(sphere_radius)):
            dist_squared = (X - sphere_center[i][0])**2 + (Y - sphere_center[i][1])**2 + (Z - sphere_center[i][2])**2
            sphere_mask = dist_squared <= sphere_radius[i]**2
            # occupancy_grid[sphere_mask] = 1

        # Create mask for cilinder
        cyl_mask = (
            ((X - self.cyl_center_xy[0])**2 + (Y - self.cyl_center_xy[1])**2) <= self.cyl_radius**2
        ) & (Z >= self.z_min) & (Z <= self.z_max)
        occupancy_grid[cyl_mask] = 1

        # Create mask for mobile manipulator base using the oriented box from
        # mobile_base_geometry.yaml. The turret_link→arm_base transform is static,
        # so this fixed box is correct for both arm and full modes.
        inflation = self.simplified_mobile_base_box_inflation
        solid = SolidPrimitive()
        solid.type = SolidPrimitive.BOX
        solid.dimensions = [
            float(self.arm_mode_base_raw_box_dims[0] + 2 * inflation[0]),
            float(self.arm_mode_base_raw_box_dims[1] + 2 * inflation[1]),
            float(self.arm_mode_base_raw_box_dims[2] + 2 * inflation[2]),
        ]
        pose = Pose()
        c = self.arm_mode_base_box_center
        pose.position.x = float(c[0])
        pose.position.y = float(c[1])
        pose.position.z = float(c[2])
        q = R.from_euler('z', self.arm_mode_base_box_rotation_z_deg, degrees=True).as_quat()
        pose.orientation.x = float(q[0])
        pose.orientation.y = float(q[1])
        pose.orientation.z = float(q[2])
        pose.orientation.w = float(q[3])

        base_mask = self._create_urdf_base_mask(X, Y, Z, solid, pose)
        occupancy_grid[base_mask] = 1
        self.get_logger().info(
            f"✅ Using oriented box for occupancy grid ({self.mode} mode): "
            f"dims={[round(d, 4) for d in solid.dimensions]}m, "
            f"centre={[round(v, 4) for v in c]}m, "
            f"rotation_z={self.arm_mode_base_box_rotation_z_deg}°, "
            f"inflation={inflation}"
        )

        if self._should_publish_planner_markers():
            # Publish obstacle markers for visualization
            self.publish_cylinder_marker(self.cyl_center_xy, self.cyl_radius, self.z_min, self.z_max)

            # Publish base marker (static oriented box, same for arm and full modes)
            self.publish_base_footprint_marker(self.base_footprint_xy, self.base_z_min, self.base_z_max)

            self.publish_wall()

        # Define the dilation distance (in meters)
        dilation_distance = 0.05  # Enlarge obstacles by 0.X meters in all directions

        # Apply dilation
        self.get_logger().info(f"Applying obstacle dilation to grid shape {occupancy_grid.shape}...")
        occupancy_grid_dilated_base = dilate_obstacles(occupancy_grid, dilation_distance, self.x_vals)
        self.get_logger().info("Obstacle dilation computed once for the static occupancy grid.")
        if self.visualize_dilated_grid:
            self.publish_dilated_grid_marker(occupancy_grid_dilated_base)
        # occupancy_grid_dilated = np.zeros(self.grid_shape, dtype=np.uint8)      # Eliminating all obstacles for now

        # Initialize collision waypoint tracking for replanning
        self.get_logger().info("Initializing replanning loop...")
        # Dict mapping grid_idx -> num_layers (int)
        # Number of neighbor layers to mark around this collision cell
        # Boundary collisions use 0 layers (cell only), interior collisions use configured value
        collision_waypoints_to_avoid = {}
        last_collision_batch_layers = {}
        last_collision_batch_center_only_retry_used = False
        MAX_REPLANNING_ATTEMPTS = 350  # Increased to allow multiple boundary narrowings (each gets 15 attempts) + grid resets
        replanning_attempt = 0
        
        # Track validated collision-free segments across replanning iterations
        # Structure: list of {'waypoints': [(world_pos, orientation, grid_idx)...], 'joint_solutions': [q...]}
        validated_start_segment = None  # Segment connected to robot current pose
        validated_goal_segment = None   # Segment connected to goal pose
        
        # Track what needs to be planned: initially from current pose to goal
        current_planning_start_pose = start_pos_wrist3
        current_planning_start_orientation = start_orientation_wrist3.as_matrix()
        current_planning_start_grid = start_idx
        current_planning_start_joint_config = None  # Will be set from current robot state
        
        current_planning_goal_pose = goal_pos_wrist3
        current_planning_goal_orientation = goal_orientation_wrist3.as_matrix()
        current_planning_goal_grid = goal_idx
        
        # Track previous boundaries to detect when gap narrows (for clearing collision waypoints)
        previous_planning_start_grid = None
        previous_planning_goal_grid = None
        
        # Detect stuck condition: same boundaries with no progress
        iterations_at_same_boundaries = 0
        MAX_ITERATIONS_SAME_BOUNDARIES = 50  # More attempts before reset (increased from 7 to explore more alternatives)
        
        # Track grid reset attempts for stuck recovery
        grid_reset_attempts = 0
        MAX_GRID_RESET_ATTEMPTS = 2  # Reduced since we have more iterations per reset
        
        # Track boundary pairs we've already tried with clean grids (to avoid pointless resets)
        # Set of tuples: (start_grid_idx, goal_grid_idx)
        tried_clean_boundaries = set()
        seen_replanning_states = set()
        allow_repeated_replanning_state_once = False
        
        self._log_hot_loop_info(lambda: f"Starting replanning loop (max {MAX_REPLANNING_ATTEMPTS} attempts)...")
        # Main planning loop with replanning support
        while replanning_attempt < MAX_REPLANNING_ATTEMPTS:
            if self._abort_if_requested(plan_generation, "goal was cancelled during replanning"):
                return

            self._log_hot_loop_info(lambda: f"Replanning iteration {replanning_attempt}...")
            
            # Initialize previous boundaries on first iteration
            if replanning_attempt == 0:
                previous_planning_start_grid = current_planning_start_grid
                previous_planning_goal_grid = current_planning_goal_grid
                # Mark initial boundaries as tried with clean grid (iteration 0 always has clean grid)
                tried_clean_boundaries.add((current_planning_start_grid, current_planning_goal_grid))
            
            # Start from the static dilated occupancy grid and overlay the current
            # collision-waypoint exclusions for this replanning iteration.
            occupancy_grid_dilated = occupancy_grid_dilated_base.copy()
            
            if replanning_attempt > 0:
                self._log_hot_loop_info(lambda: (
                    f"🔄 Replanning attempt {replanning_attempt}/{MAX_REPLANNING_ATTEMPTS - 1}: "
                    f"Avoiding {len(collision_waypoints_to_avoid)} waypoints with collisions"
                ))
                
            # Mark collision waypoints (and optionally their neighbors) as obstacles in the dilated grid
            total_cells_marked = 0
            cells_already_occupied = 0
            for grid_idx, num_layers in collision_waypoints_to_avoid.items():
                i, j, k = grid_idx
                # Check if cell is already occupied by environment obstacle
                if (0 <= i < occupancy_grid_dilated.shape[0] and
                    0 <= j < occupancy_grid_dilated.shape[1] and
                    0 <= k < occupancy_grid_dilated.shape[2]):
                    current_value = occupancy_grid_dilated[i, j, k]
                    if current_value == 1:
                        # Cell is already an obstacle from environment - can't replan through it anyway
                        cells_already_occupied += 1
                        self.get_logger().debug(
                            f"Skipped marking {grid_idx} - already occupied by environment obstacle"
                        )
                        continue
                
                # Mark cell and neighbors based on num_layers
                cells_marked = self._mark_cell_and_neighbors_as_obstacles(
                    occupancy_grid_dilated, i, j, k, num_layers
                )
                total_cells_marked += cells_marked
                
                if num_layers == 0:
                    self.get_logger().debug(
                        f"Marked grid cell {grid_idx} as obstacle (no neighbors)"
                    )
                else:
                    self.get_logger().debug(
                        f"Marked grid cell {grid_idx} with {num_layers} neighbor layer(s) "
                        f"({cells_marked} total cells)"
                    )
            
            if replanning_attempt > 0 and (total_cells_marked > 0 or cells_already_occupied > 0):
                msg = f"🚫 Marked {total_cells_marked} grid cells as obstacles ({len(collision_waypoints_to_avoid)} waypoints)"
                if cells_already_occupied > 0:
                    msg += f"\n   ⚠️  {cells_already_occupied} waypoints already at environment obstacles (unreachable!)"
                self._log_hot_loop_info(msg)
                self._maybe_log_wrist3_replanning_state(
                    'grid update',
                    current_planning_start_grid,
                    current_planning_goal_grid,
                    current_planning_start_pose,
                    current_planning_goal_pose,
                    tracked_waypoint_cells=len(collision_waypoints_to_avoid),
                    newly_blocked_voxels=total_cells_marked,
                    already_env_occupied=cells_already_occupied,
                )
            
            # Note: Visualization happens after collision detection (line ~2613) to reduce message rate
            
            # Diagnostic: verify specific cells are marked
            if collision_waypoints_to_avoid and replanning_attempt > 1:
                sample_grid_idx = list(collision_waypoints_to_avoid.keys())[0]
                si, sj, sk = sample_grid_idx
                cell_value = occupancy_grid_dilated[si, sj, sk]
                self._log_hot_loop_info(lambda: (
                    f"🔍 Verification: grid cell {sample_grid_idx} has value {cell_value} "
                    f"({'OBSTACLE' if cell_value == 1 else 'FREE' if cell_value == 0 else 'UNKNOWN'})"
                ))
            
            # --- Pre-flight check: verify start and goal are not inside obstacles ---
            self._log_hot_loop_info(lambda: f"Performing pre-flight checks (start_idx={start_idx}, goal_idx={goal_idx})...")
            start_i, start_j, start_k = start_idx
            goal_i, goal_j, goal_k = goal_idx

            self._log_hot_loop_info("Checking if indices within bounds...")
            # Check if indices are within bounds
            if not start_in_bounds:
                self._fail_current_goal(
                    f"❌ Start wrist_3 position OUT OF BOUNDS: grid_idx={start_idx}, "
                    f"pos_wrist3={start_pos_wrist3}. Start IK target is outside reachable workspace."
                )
                return
            self._log_hot_loop_info("Start indices OK. Checking goal bounds...")
                    
            if not goal_in_bounds:
                self._fail_current_goal(
                f"❌ Goal wrist_3 position OUT OF BOUNDS: grid_idx={goal_idx}, "
                f"pos_wrist3={goal_pos_wrist3}. "
                f"Goal IK target is outside reachable workspace - cannot plan path.\n"
                f"  Note: Tool0 was at {goal_pos} (may also be out of bounds)."
                )
                return
            # Check if start wrist_3 is inside an obstacle
            # IMPORTANT: Skip this check during replanning - robot is already at start position
            if replanning_attempt == 0:
                self._log_hot_loop_info("Goal indices OK. Checking if start is inside obstacle...")
                if occupancy_grid_dilated[start_i, start_j, start_k] == 1:
                    # Identify which obstacle
                    self._log_hot_loop_info("Start is inside obstacle! Identifying...")
                    self._log_hot_loop_info("Calling _fail_current_goal...")
                    self._fail_current_goal(
                        self._format_endpoint_block_message(
                            endpoint_scope="Global request start",
                            frame_label="wrist_3",
                            position=start_pos_wrist3,
                            paired_frame_label="tool0",
                            paired_position=start_pos,
                            grid_idx=start_idx,
                            occupancy_grid=occupancy_grid,
                            occupancy_grid_dilated_base=occupancy_grid_dilated_base,
                            occupancy_grid_dilated=occupancy_grid_dilated,
                            collision_waypoints_to_avoid=collision_waypoints_to_avoid,
                            replanning_attempt=replanning_attempt,
                            current_planning_start_grid=current_planning_start_grid,
                            current_planning_goal_grid=current_planning_goal_grid,
                        )
                    )
                    self._log_hot_loop_info("_fail_current_goal returned, exiting callback")
                    return
            else:
                self._log_hot_loop_info(lambda: (
                    f"Goal indices OK. Skipping start obstacle check during replanning "
                    f"(robot is already at start position)"
                ))
            
            # IMPORTANT: Global request endpoint checks should only run on the first iteration.
            # During replanning, the grid includes waypoint-exclusion overlays that can legitimately
            # cover the original request endpoints without meaning the real start/goal is invalid.
            if replanning_attempt == 0:
                # Check if start tool0 is inside an obstacle (if within grid bounds)
                if start_tool0_in_bounds:
                    st_i, st_j, st_k = start_tool0_idx
                    if occupancy_grid_dilated[st_i, st_j, st_k] == 1:
                        self._fail_current_goal(
                            self._format_endpoint_block_message(
                                endpoint_scope="Global request start",
                                frame_label="tool0",
                                position=start_pos,
                                paired_frame_label="wrist_3",
                                paired_position=start_pos_wrist3,
                                grid_idx=start_tool0_idx,
                                occupancy_grid=occupancy_grid,
                                occupancy_grid_dilated_base=occupancy_grid_dilated_base,
                                occupancy_grid_dilated=occupancy_grid_dilated,
                                collision_waypoints_to_avoid=collision_waypoints_to_avoid,
                                replanning_attempt=replanning_attempt,
                                current_planning_start_grid=current_planning_start_grid,
                                current_planning_goal_grid=current_planning_goal_grid,
                            )
                        )
                        return

                # Check if goal wrist_3 is inside an obstacle
                if occupancy_grid_dilated[goal_i, goal_j, goal_k] == 1:
                    self._fail_current_goal(
                        self._format_endpoint_block_message(
                            endpoint_scope="Global request goal",
                            frame_label="wrist_3",
                            position=goal_pos_wrist3,
                            paired_frame_label="tool0",
                            paired_position=goal_pos,
                            grid_idx=goal_idx,
                            occupancy_grid=occupancy_grid,
                            occupancy_grid_dilated_base=occupancy_grid_dilated_base,
                            occupancy_grid_dilated=occupancy_grid_dilated,
                            collision_waypoints_to_avoid=collision_waypoints_to_avoid,
                            replanning_attempt=replanning_attempt,
                            current_planning_start_grid=current_planning_start_grid,
                            current_planning_goal_grid=current_planning_goal_grid,
                        )
                    )
                    return
                
                # Check if goal tool0 is inside an obstacle (if within grid bounds)
                if goal_tool0_in_bounds:
                    gt_i, gt_j, gt_k = goal_tool0_idx
                    if occupancy_grid_dilated[gt_i, gt_j, gt_k] == 1:
                        self._fail_current_goal(
                            self._format_endpoint_block_message(
                                endpoint_scope="Global request goal",
                                frame_label="tool0",
                                position=goal_pos,
                                paired_frame_label="wrist_3",
                                paired_position=goal_pos_wrist3,
                                grid_idx=goal_tool0_idx,
                                occupancy_grid=occupancy_grid,
                                occupancy_grid_dilated_base=occupancy_grid_dilated_base,
                                occupancy_grid_dilated=occupancy_grid_dilated,
                                collision_waypoints_to_avoid=collision_waypoints_to_avoid,
                                replanning_attempt=replanning_attempt,
                                current_planning_start_grid=current_planning_start_grid,
                                current_planning_goal_grid=current_planning_goal_grid,
                            )
                        )
                        return
            else:
                self._log_hot_loop_info(
                    "Skipping global request endpoint obstacle checks during replanning because the current planning grid includes waypoint-exclusion overlays."
                )

            # Run A* with DUAL-FRAME collision checking (both wrist_3 AND tool0)
            # Plan between current planning boundaries (may be full path or just a gap)
            if self._abort_if_requested(plan_generation, "goal was cancelled before A* path search"):
                return
            
            # Diagnostic: check obstacle grid status
            total_env_obstacles = int(np.sum(occupancy_grid_dilated_base == 1))
            total_marked_obstacles = int(np.sum(
                (occupancy_grid_dilated == 1) & (occupancy_grid_dilated_base == 0)
            ))
            total_all_obstacles = int(np.sum(occupancy_grid_dilated == 1))

            # Warn only when collision waypoints exist but none of them added new obstacle cells.
            if collision_waypoints_to_avoid and total_marked_obstacles == 0 and cells_already_occupied < len(collision_waypoints_to_avoid):
                self.get_logger().warn(
                    f"⚠️  Collision waypoint tracking has {len(collision_waypoints_to_avoid)} entries, "
                    f"but none added new obstacle cells in the grid."
                )
            
            # Visualize gap boundaries AND collision waypoints in RViz
            gap_start_world = grid_to_world(*current_planning_start_grid, self.x_vals, self.y_vals, self.z_vals)
            gap_goal_world = grid_to_world(*current_planning_goal_grid, self.x_vals, self.y_vals, self.z_vals)

            publish_replanning_visualization = self._should_publish_replanning_visualization()
            if publish_replanning_visualization:
                self._log_hot_loop_info(lambda: (
                    f"📊 Visualization: Publishing {len(collision_waypoints_to_avoid)} collision waypoints to RViz "
                    f"(Grid has {total_marked_obstacles} newly marked collision cells)"
                ))
                self._publish_gap_boundary_markers(gap_start_world, gap_goal_world, collision_waypoints_to_avoid)
            
            self._log_hot_loop_info(lambda: (
                f"Running A* pathfinding with collision mode '{self.astar_collision_mode}'...\n"
                f"  From: {current_planning_start_grid} → {current_planning_goal_grid}\n"
                f"  Obstacles in grid: {total_all_obstacles} cells ({total_env_obstacles} environment + {total_marked_obstacles} collision waypoints)"
            ))

            replanning_state_signature = (
                tuple(current_planning_start_grid),
                tuple(current_planning_goal_grid),
                tuple(sorted((tuple(grid_idx), int(num_layers)) for grid_idx, num_layers in collision_waypoints_to_avoid.items())),
            )
            if replanning_state_signature in seen_replanning_states:
                if allow_repeated_replanning_state_once:
                    self.get_logger().warn(
                        "Allowing one repeated replanning state after collapsing the last collision batch to center-only."
                    )
                    allow_repeated_replanning_state_once = False
                else:
                    repeated_waypoint_samples = [grid_idx for grid_idx, _ in replanning_state_signature[2][:3]]
                    self._fail_current_goal(
                        f"❌ Path planning repeated an identical replanning state before A* search\n"
                        f"   Boundaries: {current_planning_start_grid} → {current_planning_goal_grid}\n"
                        f"   Collision waypoints: {len(replanning_state_signature[2])} (sample: {repeated_waypoint_samples})\n"
                        f"   The planner is about to rerun A* with the same inputs, so the result would repeat.\n"
                        f"   Suggestion: Goal likely unreachable through this gap - try a different goal or robot starting pose."
                    )
                    return
            seen_replanning_states.add(replanning_state_signature)
            
            # A* searches between current planning boundaries (initially full path, later narrowed gap)
            # Grid dimensions are always the full workspace - only start/goal positions change
            path_stats = {}
            path = find_path(
                occupancy_grid_dilated,
                current_planning_start_grid,
                current_planning_goal_grid,
                x_vals=self.x_vals,
                y_vals=self.y_vals,
                z_vals=self.z_vals,
                T_wrist3_tool0=self._T_wrist3_tool0,
                start_orientation=current_planning_start_orientation,
                goal_orientation=current_planning_goal_orientation,
                astar_collision_mode=self.astar_collision_mode,
                stats_out=path_stats,
                cylinder_center_xy=self.cyl_center_xy,
                cylinder_radius=self.cyl_radius,
                cylinder_z_range=(self.z_min, self.z_max)
            )
            self._record_astar_search_metrics(path_stats)
            self._log_hot_loop_info(lambda: (
                f"Found path with length {len(path)} "
                f"(A* mode: {path_stats.get('collision_mode', self.astar_collision_mode)}, "
                f"orientation_filter={'on' if path_stats.get('dual_frame_cache_enabled', 0) else 'off'})"
            ))
            
            # Diagnostic: check if A* path goes through any marked collision waypoints
            if replanning_attempt > 0 and path and collision_waypoints_to_avoid:
                path_grid_indices = set()
                for waypoint in path:
                    # Convert waypoint to grid index
                    wp_i = np.argmin(np.abs(self.x_vals - waypoint[0]))
                    wp_j = np.argmin(np.abs(self.y_vals - waypoint[1]))
                    wp_k = np.argmin(np.abs(self.z_vals - waypoint[2]))
                    path_grid_indices.add((wp_i, wp_j, wp_k))
                
                # Check for intersection with collision waypoints
                collision_grids = set(collision_waypoints_to_avoid.keys())
                path_through_marked = path_grid_indices.intersection(collision_grids)
                if path_through_marked:
                    self.get_logger().warn(
                        f"⚠️ A* path goes through {len(path_through_marked)} marked collision waypoints: {list(path_through_marked)[:3]}"
                        f"\n   This suggests gap is too narrow - no alternative path exists!"
                    )
            
            if not path:
                expanded_last_batch_cells = [
                    grid_idx
                    for grid_idx, num_layers in last_collision_batch_layers.items()
                    if num_layers > 0 and collision_waypoints_to_avoid.get(grid_idx, 0) > 0
                ]
                if expanded_last_batch_cells and not last_collision_batch_center_only_retry_used:
                    for grid_idx in expanded_last_batch_cells:
                        collision_waypoints_to_avoid[grid_idx] = 0
                    last_collision_batch_center_only_retry_used = True
                    allow_repeated_replanning_state_once = True
                    self.get_logger().warn(
                        f"No A* path after expanded collision marking for gap {current_planning_start_grid} → {current_planning_goal_grid}. "
                        f"Retrying once with {len(expanded_last_batch_cells)} last-batch waypoint cell(s) forced to center-only."
                    )
                    self._maybe_log_wrist3_replanning_state(
                        'center-only retry',
                        current_planning_start_grid,
                        current_planning_goal_grid,
                        current_planning_start_pose,
                        current_planning_goal_pose,
                        retried_waypoint_cells=len(expanded_last_batch_cells),
                        tracked_waypoint_cells=len(collision_waypoints_to_avoid),
                    )
                    continue

                self.get_logger().warn(
                    "No path found by A* algorithm!\n"
                    f"  A* collision mode: {path_stats.get('collision_mode', self.astar_collision_mode)}\n"
                    f"  Start: {current_planning_start_pose} (grid {current_planning_start_grid})\n"
                    f"  Goal: {current_planning_goal_pose} (grid {current_planning_goal_grid})\n"
                    "  Possible causes: goal unreachable due to obstacles blocking all paths, "
                    "or start/goal in narrow passage that was eliminated by dilation."
                )
                with self._state_lock:
                    self.execution_complete = True  # Mark execution as complete to allow new goals
                self.publish_planner_goal_failed(True)
                self._log_plan_metrics('astar_no_path')
                return

            # Convert path to world coordinates
            path_world_raw = [grid_to_world(i, j, k, self.x_vals, self.y_vals, self.z_vals) for i, j, k in path]
            # Keep grid indices for distance-based orientation calculation (matches A*)
            path_grid_indices = path  # Store grid indices before pruning

            # Debug: publish raw path (pre-prune)
            if publish_replanning_visualization:
                self.publish_ee_path_markers(
                    path_world_raw,
                    frame_id="arm_base",
                    current_ee_xyz=start_pos_wrist3,
                    goal_xyz=goal_pos_wrist3,
                    ns="ee_path_raw",
                    id_offset=100,
                    rgba=(0.0, 1.0, 1.0, 0.5),   # cyan
                    z_offset=0.002,              # 2mm above
                    line_width=0.01,             # thicker
                    sphere_diam=0.03             # bigger
                )

            # Prune path endpoints
            path_world, path_trim_start, path_trim_end = self._prune_path_endpoints(
                path_world_raw,
                start_pos_wrist3,
                goal_pos_wrist3,
                tol_m=0.02,
                log=self.hot_loop_info_logging,
                return_trim=True,
            )
            path_len = len(path_world)
            if path_len == 0:
                self.get_logger().warn("Path pruned to empty. Falling back to direct goal IK.")

            # Interpolate orientations along the (possibly pruned) path (wrist_3 orientations)
            # CRITICAL: Use DISTANCE-BASED interpolation to match A* orientation computation
            # (A* uses Euclidean distance in GRID SPACE from start, not waypoint index)
            # We use the UNPRUNED path distances to match A* exactly, then subset for pruned path

            # First, compute orientations for ALL waypoints in the unpruned path (matches A* validation)
            start_idx_array = np.array(current_planning_start_grid, dtype=float)
            goal_idx_array = np.array(current_planning_goal_grid, dtype=float)
            max_dist_grid = np.linalg.norm(goal_idx_array - start_idx_array)  # Distance in grid index space

            key_rots = R.from_quat([R.from_matrix(current_planning_start_orientation).as_quat(), R.from_matrix(current_planning_goal_orientation).as_quat()])
            slerp = Slerp([0, 1], key_rots)

            # Compute orientation for each waypoint in UNPRUNED path using grid-space distances
            unpruned_orientations = []
            for grid_idx in path_grid_indices:
                dist_from_start_grid = np.linalg.norm(np.array(grid_idx, dtype=float) - start_idx_array)
                progress = min(dist_from_start_grid / max_dist_grid, 1.0) if max_dist_grid > 0 else 0.0
                orientation = slerp([progress])[0].as_matrix()
                unpruned_orientations.append(orientation)

            # Match pruned waypoints to their unpruned counterparts to get consistent orientations
            if path_len == 0:
                # No waypoint orientations needed; we will only use the goal orientation
                interp_rot_matrices = np.array([R.from_matrix(current_planning_goal_orientation).as_matrix()])
                pruned_path_grid_indices = []
            else:
                pruned_end_index = len(path_grid_indices) - path_trim_end if path_trim_end > 0 else len(path_grid_indices)
                pruned_path_grid_indices = list(path_grid_indices[path_trim_start:pruned_end_index])
                interp_rot_matrices = np.array(unpruned_orientations[path_trim_start:pruned_end_index])

                self._log_hot_loop_info(lambda: (
                    f"Orientation interpolation: using {len(interp_rot_matrices)} orientations "
                    f"from unpruned path (A*-consistent) for {path_len} pruned waypoints"
                ))

            validate_tool0_path_post_astar = self.validate_post_astar_tool0_path
            if publish_replanning_visualization or validate_tool0_path_post_astar:
                # Build full path lists for visualization (waypoints + final goal)
                if path_len == 0:
                    _vis_rots = [goal_orientation_wrist3.as_matrix()]
                else:
                    _vis_rots = list(interp_rot_matrices) + [goal_orientation_wrist3.as_matrix()]
                _vis_wrist3 = list(path_world) + [tuple(goal_pos_wrist3)]

                # Compute tool0 positions by applying T_wrist3_tool0 at each wrist_3 pose
                _vis_tool0 = []
                for _pf, _rf in zip(_vis_wrist3, _vis_rots):
                    _T = np.eye(4)
                    _T[:3, :3] = _rf
                    _T[:3, 3] = np.array(_pf)
                    _vis_tool0.append(tuple((_T @ self._T_wrist3_tool0)[:3, 3]))

                if validate_tool0_path_post_astar:
                    # --- SAFETY CHECK: Validate tool0 path (should never fail since A* checked it) ---
                    # This is a redundant check since A* now validates both frames during search,
                    # but can be re-enabled for debugging numerical/rounding issues.
                    tool0_collision_detected = False
                    tool0_out_of_bounds_count = 0
                    for idx, tool0_pos in enumerate(_vis_tool0):
                        if self._abort_if_requested(plan_generation, "goal was cancelled during tool0 path validation"):
                            return

                        tool0_grid_idx, tool0_in_bounds = world_to_grid_checked(
                            *tool0_pos, self.x_vals, self.y_vals, self.z_vals
                        )
                        ti, tj, tk = tool0_grid_idx

                        if not tool0_in_bounds:
                            tool0_out_of_bounds_count += 1
                            continue

                        if occupancy_grid_dilated[ti, tj, tk] == 1:
                            obstacle_name = self._identify_obstacle(tool0_pos, occupancy_grid, tool0_grid_idx)

                            if idx < len(pruned_path_grid_indices):
                                grid_idx_for_waypoint = pruned_path_grid_indices[idx]
                                dist_from_start_grid = np.linalg.norm(
                                    np.array(grid_idx_for_waypoint, dtype=float) - np.array(start_idx, dtype=float)
                                )
                                max_dist_grid = np.linalg.norm(
                                    np.array(goal_idx, dtype=float) - np.array(start_idx, dtype=float)
                                )
                                a_star_progress = min(dist_from_start_grid / max_dist_grid, 1.0) if max_dist_grid > 0 else 0.0
                            else:
                                a_star_progress = 1.0

                            self._fail_current_goal(
                                f"❌ UNEXPECTED: Tool0 waypoint {idx} COLLIDES despite dual-frame A*: {obstacle_name}\n"
                                f"  Tool0 position: {[round(x, 3) for x in tool0_pos]}\n"
                                f"  Corresponding wrist_3: {[round(x, 3) for x in _vis_wrist3[idx]]}\n"
                                f"  Grid index (tool0): {tool0_grid_idx}\n"
                                f"  A* progress metric: {a_star_progress:.4f}\n"
                                f"  This indicates a bug in A* dual-frame collision checking or orientation interpolation mismatch."
                            )
                            tool0_collision_detected = True
                            break

                    if tool0_out_of_bounds_count > 0:
                        self._log_hot_loop_info(lambda: (
                            f"ℹ️  Tool0 extends beyond workspace bounds at {tool0_out_of_bounds_count}/{len(_vis_tool0)} waypoints.\n"
                            f"  This is expected/allowed - wrist_3 (IK target) remains within reachable workspace."
                        ))

                    if tool0_collision_detected:
                        return  # Path validation failed, abort trajectory

            # Publish pruned wrist_3 path (orange)
            if publish_replanning_visualization:
                self.publish_ee_path_markers(
                    _vis_wrist3,
                    frame_id="arm_base",
                    current_ee_xyz=start_pos_wrist3,
                    goal_xyz=goal_pos_wrist3,
                    ns="ee_path_pruned",
                    id_offset=200,
                    rgba=(1.0, 0.5, 0.0, 1.0),  # orange - wrist_3 path
                    z_offset=0.0,
                    line_width=0.01,
                    sphere_diam=0.02
                )
                # Publish corresponding tool0 path (magenta)
                self.publish_ee_path_markers(
                    _vis_tool0,
                    frame_id="arm_base",
                    current_ee_xyz=start_pos,
                    goal_xyz=goal_pos,
                    ns="ee_path_tool0",
                    id_offset=300,
                    rgba=(0.8, 0.0, 0.8, 1.0),  # magenta - tool0 path
                    z_offset=0.0,
                    line_width=0.01,
                    sphere_diam=0.02
                )
            # End of A* path planning and processing

            # Plan joint values for this iteration's path
            # On first iteration: full path from robot to goal  
            # On subsequent iterations: only the gap between preserved segments
            all_joint_values = []
            all_joint_values_print = []
            waypoints_requiring_replan = []
            waypoint_collision_status = []  # Track collision status: (idx, is_free, joint_solution, grid_idx)

            # Determine which waypoints to validate this iteration
            if validated_start_segment is None and validated_goal_segment is None:
                # First iteration: validate all waypoints
                waypoints_to_validate_start = 0
                waypoints_to_validate_end = len(path_world) - 1
                self._log_hot_loop_info(lambda: f"First iteration: validating all {len(path_world)} waypoints")
                
                # Initialize q_current from robot state
                try:
                    q_current = np.array([current_joint_state.position[i] for i in joint_indices])
                    q_current_initial = q_current.copy()
                except Exception as e:
                    self._fail_current_goal(f"Failed to extract current arm joint positions: {e}")
                    return
            else:
                # Subsequent iterations: validate only new gap waypoints
                # Gap interior excludes endpoints (already validated in preserved segments)
                waypoints_to_validate_start = 0 if validated_start_segment is None else 1  # Skip first if it's the boundary
                waypoints_to_validate_end = len(path_world) - (2 if validated_goal_segment is not None else 1)
                
                self._log_hot_loop_info(lambda: (
                    f"Gap validation: checking waypoints {waypoints_to_validate_start} to {waypoints_to_validate_end} "
                    f"({waypoints_to_validate_end - waypoints_to_validate_start + 1} new waypoints)"
                ))
                
                # Start from the validated start segment's last joint configuration
                if validated_start_segment and validated_start_segment['joint_solutions']:
                    q_current = validated_start_segment['joint_solutions'][-1].copy()
                else:
                    q_current = q_current_initial.copy()
            
            all_joint_values_print.append(q_current)

            waypoint_validation_scope = (
                'full-path candidate'
                if validated_start_segment is None and validated_goal_segment is None
                else 'replanning-gap candidate'
            )

            selected_waypoint_candidates = []
            provisional_q_current = q_current.copy()
            committed_q_current = q_current.copy()

            # Process waypoints by selecting the closest IK first; collision validation happens in batch afterwards.
            for i in range(waypoints_to_validate_start, waypoints_to_validate_end + 1):
                if self._abort_if_requested(plan_generation, f"goal was cancelled while solving waypoint {i}"):
                    return

                grid_idx = (
                    pruned_path_grid_indices[i]
                    if i < len(pruned_path_grid_indices)
                    else None
                )

                T = create_pose_matrix(path_world[i], interp_rot_matrices[i])
                ik_solutions = closed_form_algorithm(
                    T,
                    q_current=provisional_q_current,
                    type=0,
                    return_all_solutions=(self.max_ik_solutions_to_test != 1),
                )
                ranked_candidates = self._get_ranked_ik_candidates(ik_solutions, provisional_q_current, waypoint_idx=i)

                if not ranked_candidates:
                    self.get_logger().error(
                        f"{waypoint_validation_scope.capitalize()} waypoint {i} at {path_world[i]} has no valid IK solution. "
                        f"Grid index: {grid_idx if grid_idx is not None else 'unknown'}"
                    )
                    waypoints_requiring_replan.append(
                        (
                            i,
                            path_world[i],
                            grid_idx,
                            "no valid IK solution",
                        )
                    )
                    waypoint_collision_status.append((i, False, None, grid_idx))
                    continue

                q_reference = provisional_q_current.copy()
                closest_idx, q_new, _ = ranked_candidates[0]
                q_new = self._normalize_joint_solution(q_new, q_reference)

                selected_waypoint_candidates.append({
                    'waypoint_idx': i,
                    'grid_idx': grid_idx,
                    'ik_solutions': ik_solutions,
                    'q_reference': q_reference,
                    'q_candidate': q_new,
                    'closest_idx': closest_idx,
                    'fallback_candidates': ranked_candidates[1:],
                })
                provisional_q_current = q_new

            if self.enable_collision_checking and selected_waypoint_candidates:
                collision_statuses = self._check_collision_batch_sync(
                    [candidate['q_candidate'] for candidate in selected_waypoint_candidates],
                    publish_visualization_index=0 if self.visualize_collision_checks else None,
                    timeout_sec=2.0,
                    cancel_check=lambda: self._should_abort_plan(plan_generation),
                )

                rerank_remaining_waypoints = False
                for candidate, collision_status in zip(selected_waypoint_candidates, collision_statuses):
                    waypoint_idx = candidate['waypoint_idx']
                    grid_idx = candidate['grid_idx']

                    if not rerank_remaining_waypoints and np.allclose(candidate['q_reference'], committed_q_current, atol=1e-9):
                        q_reference = candidate['q_reference']
                        q_new = candidate['q_candidate']
                        closest_idx = candidate['closest_idx']
                        fallback_candidates = candidate['fallback_candidates']
                    else:
                        rerank_remaining_waypoints = True
                        q_reference = committed_q_current.copy()
                        ranked_candidates = self._get_ranked_ik_candidates(
                            candidate['ik_solutions'],
                            committed_q_current,
                            waypoint_idx=waypoint_idx,
                        )

                        if not ranked_candidates:
                            self.get_logger().error(
                                f"{waypoint_validation_scope.capitalize()} waypoint {waypoint_idx} at {path_world[waypoint_idx]} has no valid IK solution. "
                                f"Grid index: {grid_idx if grid_idx is not None else 'unknown'}"
                            )
                            waypoints_requiring_replan.append(
                                (
                                    waypoint_idx,
                                    path_world[waypoint_idx],
                                    grid_idx,
                                    "no valid IK solution",
                                )
                            )
                            waypoint_collision_status.append((waypoint_idx, False, None, grid_idx))
                            continue

                        closest_idx, q_new, _ = ranked_candidates[0]
                        q_new = self._normalize_joint_solution(q_new, q_reference)
                        fallback_candidates = ranked_candidates[1:]
                        collision_status = self._check_collision_sync(
                            q_new,
                            publish_visualization=False,
                            timeout_sec=2.0,
                            cancel_check=lambda: self._should_abort_plan(plan_generation),
                        )

                    if collision_status == CollisionCheckStatus.CHECK_FAILED:
                        self._fail_current_goal(
                            f"{waypoint_validation_scope.capitalize()} waypoint {waypoint_idx} could not be collision-validated. "
                            f"Aborting instead of replanning on uncertain data."
                        )
                        return

                    if collision_status == CollisionCheckStatus.COLLISION:
                        if fallback_candidates:
                            fallback_solution, fallback_outcome = self._select_collision_free_ranked_candidate(
                                fallback_candidates,
                                waypoint_idx=waypoint_idx,
                                timeout_sec=2.0,
                                cancel_check=lambda: self._should_abort_plan(plan_generation),
                            )
                            if fallback_outcome == IKSelectionOutcome.CHECK_FAILED:
                                self._fail_current_goal(
                                    f"{waypoint_validation_scope.capitalize()} waypoint {waypoint_idx} fallback IK solution could not be collision-validated. "
                                    f"Aborting instead of replanning on uncertain data."
                                )
                                return
                            if fallback_solution is not None:
                                fallback_solution = self._normalize_joint_solution(fallback_solution, q_reference)

                                all_joint_values.append(fallback_solution)
                                all_joint_values_print.append(fallback_solution)
                                waypoint_collision_status.append((waypoint_idx, True, fallback_solution, grid_idx))
                                committed_q_current = fallback_solution.copy()
                                if not np.allclose(fallback_solution, q_new, atol=1e-9):
                                    rerank_remaining_waypoints = True
                                continue

                        if self.log_detailed_planning_diagnostics:
                            self.get_logger().error(
                                f"{waypoint_validation_scope.capitalize()} waypoint {waypoint_idx} at {path_world[waypoint_idx]} has closest IK solution {closest_idx} in collision. "
                                f"Grid index: {grid_idx if grid_idx is not None else 'unknown'}"
                            )
                        waypoints_requiring_replan.append(
                            (
                                waypoint_idx,
                                path_world[waypoint_idx],
                                grid_idx,
                                'closest IK solution in collision',
                            )
                        )
                        waypoint_collision_status.append((waypoint_idx, False, None, grid_idx))
                        rerank_remaining_waypoints = True
                        continue

                    all_joint_values.append(q_new)
                    all_joint_values_print.append(q_new)
                    waypoint_collision_status.append((waypoint_idx, True, q_new, grid_idx))
                    committed_q_current = q_new.copy()
            else:
                for candidate in selected_waypoint_candidates:
                    waypoint_idx = candidate['waypoint_idx']
                    q_new = candidate['q_candidate']
                    grid_idx = candidate['grid_idx']
                    all_joint_values.append(q_new)
                    all_joint_values_print.append(q_new)
                    waypoint_collision_status.append((waypoint_idx, True, q_new, grid_idx))
                    committed_q_current = q_new.copy()

            q_current = committed_q_current.copy()
            
            # After processing waypoints, analyze segments and update preserved segments
            if waypoint_collision_status and waypoints_requiring_replan:
                # Segment the path based on collision status
                segments = self._segment_path_by_collision(waypoint_collision_status)
                
                if segments:
                    # Find collision-free segments at the boundaries (connected to start/goal)
                    first_segment = segments[0] if segments else None
                    last_segment = segments[-1] if segments else None
                    
                    # Log segment analysis
                    self._log_hot_loop_info(lambda: f"Path segment analysis: {len(segments)} segment(s) found")
                    
                    # Update/extend start-connected segment if present
                    if first_segment and first_segment['is_collision_free']:
                        if validated_start_segment is None:
                            # First iteration: store initial start segment
                            validated_start_segment = {
                                'waypoints': [(path_world[wp_idx], interp_rot_matrices[wp_idx], pruned_path_grid_indices[wp_idx]) 
                                              for wp_idx in first_segment['waypoints']],
                                'joint_solutions': first_segment['joint_solutions']
                            }
                            self._log_hot_loop_info(lambda: (
                                f"✅ Preserved start segment: {len(validated_start_segment['waypoints'])} waypoints "
                                f"(from robot pose)"
                            ))
                        else:
                            # Subsequent iterations: extend start segment with validated gap waypoints
                            new_waypoints = [(path_world[wp_idx], interp_rot_matrices[wp_idx], pruned_path_grid_indices[wp_idx]) 
                                            for wp_idx in first_segment['waypoints']]
                            validated_start_segment['waypoints'].extend(new_waypoints[1:])  # Skip first (boundary point)
                            validated_start_segment['joint_solutions'].extend(first_segment['joint_solutions'][1:])
                            self._log_hot_loop_info(lambda: (
                                f"✅ Extended start segment: added {len(new_waypoints) - 1} waypoints "
                                f"(total: {len(validated_start_segment['waypoints'])} waypoints)"
                            ))
                    
                    # Update/extend goal-connected segment if present
                    if last_segment and last_segment['is_collision_free']:
                        if validated_goal_segment is None:
                            # First iteration: store initial goal segment
                            validated_goal_segment = {
                                'waypoints': [(path_world[wp_idx], interp_rot_matrices[wp_idx], pruned_path_grid_indices[wp_idx]) 
                                             for wp_idx in last_segment['waypoints']],
                                'joint_solutions': last_segment['joint_solutions']
                            }
                            self._log_hot_loop_info(lambda: (
                                f"✅ Preserved goal segment: {len(validated_goal_segment['waypoints'])} waypoints "
                                f"(to goal pose)"
                            ))
                        else:
                            # Subsequent iterations: prepend validated gap waypoints to goal segment
                            new_waypoints = [(path_world[wp_idx], interp_rot_matrices[wp_idx], pruned_path_grid_indices[wp_idx]) 
                                            for wp_idx in last_segment['waypoints']]
                            validated_goal_segment['waypoints'] = new_waypoints[:-1] + validated_goal_segment['waypoints']  # Skip last (boundary)
                            validated_goal_segment['joint_solutions'] = last_segment['joint_solutions'][:-1] + validated_goal_segment['joint_solutions']
                            self._log_hot_loop_info(lambda: (
                                f"✅ Extended goal segment: added {len(new_waypoints) - 1} waypoints "
                                f"(total: {len(validated_goal_segment['waypoints'])} waypoints)"
                            ))
                    
                    # Check if we have a complete collision-free path now
                    if len(segments) == 1 and segments[0]['is_collision_free']:
                        self._log_hot_loop_info("🎉 Complete collision-free path found!")
                        # Break out of replanning loop - path is valid
                        break
                    
                    # Still have gaps - need to replan
                    # Find the gap to replan (between validated segments or from collision segments)
                    if validated_start_segment and validated_goal_segment:
                        # Update planning boundaries to replan only the gap
                        last_start_wp = validated_start_segment['waypoints'][-1]
                        first_goal_wp = validated_goal_segment['waypoints'][0]
                        
                        current_planning_start_pose = last_start_wp[0]
                        current_planning_start_orientation = last_start_wp[1]
                        current_planning_start_grid = last_start_wp[2]
                        
                        current_planning_goal_pose = first_goal_wp[0]
                        current_planning_goal_orientation = first_goal_wp[1]
                        current_planning_goal_grid = first_goal_wp[2]
                        
                        # Check if planning boundaries actually changed (gap narrowed)
                        boundaries_changed = (
                            previous_planning_start_grid != current_planning_start_grid or
                            previous_planning_goal_grid != current_planning_goal_grid
                        )
                        
                        if boundaries_changed:
                            # Gap narrowed - clear accumulated collision waypoints for fresh start
                            old_count = len(collision_waypoints_to_avoid)
                            old_waypoints = list(collision_waypoints_to_avoid.keys())[:5]  # Sample for logging
                            collision_waypoints_to_avoid.clear()
                            # Markers will be cleared automatically on next _publish_gap_boundary_markers call
                            iterations_at_same_boundaries = 0  # Reset stuck counter
                            
                            # Track new boundaries as tried with clean grid
                            tried_clean_boundaries.add((current_planning_start_grid, current_planning_goal_grid))
                            
                            self._log_hot_loop_info(lambda: (
                                f"📍 Planning boundaries changed (gap narrowed):\n"
                                f"    New start: {[round(x, 3) for x in current_planning_start_pose]} (grid {current_planning_start_grid})\n"
                                f"    New goal:  {[round(x, 3) for x in current_planning_goal_pose]} (grid {current_planning_goal_grid})\n"
                                f"    🗑️  Cleared {old_count} collision waypoints (e.g., {old_waypoints})\n"
                                f"    ✅ Dict now has {len(collision_waypoints_to_avoid)} waypoints (should be 0)\n"
                                f"    🔄 Resetting iteration counter (was {iterations_at_same_boundaries}) - fresh start with 0/{MAX_ITERATIONS_SAME_BOUNDARIES} for new boundaries\n"
                                f"    📊 Global attempt: {replanning_attempt}/{MAX_REPLANNING_ATTEMPTS}"
                            ))
                            # Update tracked boundaries
                            previous_planning_start_grid = current_planning_start_grid
                            previous_planning_goal_grid = current_planning_goal_grid
                        else:
                            # Boundaries unchanged - keep accumulating collision waypoints
                            iterations_at_same_boundaries += 1
                            self._log_hot_loop_info(lambda: (
                                f"📍 Planning boundaries unchanged (iteration {iterations_at_same_boundaries}/{MAX_ITERATIONS_SAME_BOUNDARIES}):\n"
                                f"    Start: {[round(x, 3) for x in current_planning_start_pose]} (grid {current_planning_start_grid})\n"
                                f"    Goal:  {[round(x, 3) for x in current_planning_goal_pose]} (grid {current_planning_goal_grid})\n"
                                f"    Accumulating collision waypoints ({len(collision_waypoints_to_avoid)} existing)\n"
                                f"    📊 Global attempt: {replanning_attempt}/{MAX_REPLANNING_ATTEMPTS}"
                            ))
                            
                            # Early termination if stuck at same boundaries
                            if iterations_at_same_boundaries >= MAX_ITERATIONS_SAME_BOUNDARIES:
                                # Check if we've already tried these exact boundaries with a clean grid
                                boundary_pair = (current_planning_start_grid, current_planning_goal_grid)
                                
                                if boundary_pair in tried_clean_boundaries:
                                    # Already tried this path with clean grid - resetting won't help
                                    collision_info = list(collision_waypoints_to_avoid.keys())[:3]
                                    self._log_replanning_limit_hit(
                                        'MAX_ITERATIONS_SAME_BOUNDARIES',
                                        iterations_at_same_boundaries,
                                        MAX_ITERATIONS_SAME_BOUNDARIES,
                                        (
                                            f"Stopping at unchanged gap {current_planning_start_grid} → {current_planning_goal_grid} "
                                            "because this boundary pair already consumed its clean-grid retry."
                                        ),
                                    )
                                    self._fail_current_goal(
                                        f"❌ Path planning FAILED: Already tried boundaries {current_planning_start_grid} → {current_planning_goal_grid} with clean grid\n"
                                        f"   {len(collision_waypoints_to_avoid)} collision waypoints prevent path: {collision_info}\n"
                                        f"   Grid reset would not help (same collisions will occur).\n"
                                        f"   Suggestion: Goal unreachable from current pose - try different goal or robot starting position."
                                    )
                                    return
                                
                                # Check if we've already reset too many times
                                if grid_reset_attempts >= MAX_GRID_RESET_ATTEMPTS:
                                    collision_info = list(collision_waypoints_to_avoid.keys())[:3]
                                    self._log_replanning_limit_hit(
                                        'MAX_GRID_RESET_ATTEMPTS',
                                        grid_reset_attempts,
                                        MAX_GRID_RESET_ATTEMPTS,
                                        (
                                            f"Stopping at unchanged gap {current_planning_start_grid} → {current_planning_goal_grid} "
                                            f"after also reaching MAX_ITERATIONS_SAME_BOUNDARIES={MAX_ITERATIONS_SAME_BOUNDARIES}."
                                        ),
                                    )
                                    self._fail_current_goal(
                                        f"❌ Path planning FAILED after {replanning_attempt} attempts and {grid_reset_attempts} grid resets\n"
                                        f"   Final boundaries: {current_planning_start_grid} → {current_planning_goal_grid}\n"
                                        f"   Gap distance: {np.linalg.norm(np.array(current_planning_goal_grid) - np.array(current_planning_start_grid)):.1f} grid cells\n"
                                        f"   {len(collision_waypoints_to_avoid)} persistent collision waypoints: {collision_info}\n"
                                        f"   Multiple grid resets result in same collision waypoints.\n"
                                        f"   Suggestion: Goal likely unreachable due to kinematic constraints or narrow passage - try different goal or robot starting pose."
                                    )
                                    return
                                
                                # Stuck at narrow gap - try RESETTING the grid obstacles and starting fresh
                                grid_reset_attempts += 1
                                
                                self.get_logger().warn(
                                    f"⚠️ Stuck at narrow gap after {iterations_at_same_boundaries} iterations (reset attempt {grid_reset_attempts}/{MAX_GRID_RESET_ATTEMPTS})\n"
                                    f"   Current boundaries: {current_planning_start_grid} → {current_planning_goal_grid}\n"
                                    f"   Attempting recovery: RESETTING grid (clearing {len(collision_waypoints_to_avoid)} marked collision cells) and replanning with fresh grid"
                                )
                                
                                # Track that we've now tried these boundaries with a clean grid
                                tried_clean_boundaries.add(boundary_pair)
                                
                                # Clear collision waypoints to allow fresh search with clean grid
                                num_cleared = len(collision_waypoints_to_avoid)
                                collision_waypoints_to_avoid.clear()
                                # Markers will be cleared automatically on next _publish_gap_boundary_markers call
                                iterations_at_same_boundaries = 0
                                
                                self._log_hot_loop_info(lambda: (
                                    f"   Cleared {num_cleared} collision waypoints - starting fresh with clean grid\n"
                                    f"   Boundaries unchanged: {current_planning_start_grid} → {current_planning_goal_grid}"
                                ))
                                
                                # Continue replanning with clean grid
                                continue
                    else:
                        # First iteration or only one segment validated - keep existing collision waypoints
                        if replanning_attempt > 0:
                            iterations_at_same_boundaries += 1
                            self._log_hot_loop_info(lambda: (
                                f"📍 Continuing with current boundaries (segments not yet converged, iteration {iterations_at_same_boundaries}/{MAX_ITERATIONS_SAME_BOUNDARIES})\n"
                                f"    Accumulating collision waypoints ({len(collision_waypoints_to_avoid)} existing)\n"
                                f"    📊 Global attempt: {replanning_attempt}/{MAX_REPLANNING_ATTEMPTS}"
                            ))
                            
                            # Early termination if stuck before segments converge
                            if iterations_at_same_boundaries >= MAX_ITERATIONS_SAME_BOUNDARIES:
                                # Collect collision waypoint info for diagnostic
                                collision_info = list(collision_waypoints_to_avoid.keys())[:3]
                                self._log_replanning_limit_hit(
                                    'MAX_ITERATIONS_SAME_BOUNDARIES',
                                    iterations_at_same_boundaries,
                                    MAX_ITERATIONS_SAME_BOUNDARIES,
                                    'Stopping before start/goal segments converged to a narrowed gap.',
                                )
                                
                                self._fail_current_goal(
                                    f"❌ Path planning STUCK: Cannot converge segments after {replanning_attempt} attempts\n"
                                    f"   {len(collision_waypoints_to_avoid)} collision waypoints (e.g., {collision_info})\n"
                                    f"   Unable to find collision-free segments connecting to start/goal.\n"
                                    f"   Repeated self-collisions at same locations (turret_link <-> arm_plate_link).\n"
                                    f"   Suggestion: Goal requires different robot configuration or approach angle."
                                )
                                return
                    
                    # Mark collision waypoints as obstacles for next iteration
                    # Distinguish between boundary and interior collision waypoints:
                    # - Boundary: first TWO and last TWO waypoints in each collision segment
                    #   → Mark ONLY the cell itself (0 layers) to avoid invalidating adjacent free waypoints
                    # - Interior: waypoints in the middle of collision segment (positions 3 to N-2 for segments with 5+ waypoints)
                    #   → Mark cell with configured number of neighbor layers for stronger avoidance
                    # - Segments with ≤4 waypoints: all treated as boundaries (no neighbor marking)
                    
                    # Collect all collision waypoint indices
                    collision_waypoint_indices = {wp_idx for wp_idx, is_free, _, _ in waypoint_collision_status if not is_free}
                    
                    # Identify boundary collision waypoints (first TWO and last TWO in collision segments)
                    # Prevents marking free path cells as obstacles when boundary collision cells have neighbors marked
                    boundary_collision_indices = set()
                    for segment in segments:
                        if not segment['is_collision_free'] and len(segment['waypoints']) > 0:
                            seg_len = len(segment['waypoints'])
                            if seg_len <= 4:
                                # If segment has 4 or fewer waypoints, all are boundaries (avoid marking neighbors)
                                boundary_collision_indices.update(segment['waypoints'])
                            else:
                                # First two waypoints
                                boundary_collision_indices.add(segment['waypoints'][0])
                                boundary_collision_indices.add(segment['waypoints'][1])
                                # Last two waypoints
                                boundary_collision_indices.add(segment['waypoints'][-2])
                                boundary_collision_indices.add(segment['waypoints'][-1])
                    
                    # Add collision waypoints to avoid set with appropriate marking strategy
                    new_collision_waypoints = []
                    repeated_collision_waypoints = []
                    current_collision_batch_layers = {}
                    for wp_idx, is_free, _, grid_idx in waypoint_collision_status:
                        if not is_free and grid_idx is not None:
                            grid_tuple = tuple(grid_idx)
                            # Track if this is a new or repeated collision
                            if grid_tuple in collision_waypoints_to_avoid:
                                repeated_collision_waypoints.append(grid_tuple)
                            else:
                                new_collision_waypoints.append(grid_tuple)
                            
                            # Check if this is a boundary or interior collision waypoint
                            is_boundary = wp_idx in boundary_collision_indices
                            # Boundary collisions always stay cell-only to preserve narrow passages.
                            num_layers = 0 if is_boundary else self.collision_avoidance_neighbor_layers
                            collision_waypoints_to_avoid[grid_tuple] = num_layers
                            current_collision_batch_layers[grid_tuple] = num_layers

                    last_collision_batch_layers = current_collision_batch_layers
                    last_collision_batch_center_only_retry_used = False
                    
                    self._log_hot_loop_info(lambda: (
                        f"Identified {len(collision_waypoint_indices)} collision waypoints: "
                        f"{len(boundary_collision_indices)} boundary (first/last 2 per segment, cell-only), "
                        f"{len(collision_waypoint_indices) - len(boundary_collision_indices)} interior ({self.collision_avoidance_neighbor_layers} layers)"
                    ))
                    # Collision waypoints will be visualized on next A* iteration via gap_boundaries
                    
                    if repeated_collision_waypoints:
                        self.get_logger().warn(
                            f"⚠️  {len(repeated_collision_waypoints)} waypoints are REPEATED collisions from previous iterations: {repeated_collision_waypoints}"
                            f"\n   → A* cannot find alternative paths - gap forces path through these waypoints!"
                        )

                    self._maybe_log_wrist3_replanning_state(
                        'collision diagnostics',
                        current_planning_start_grid,
                        current_planning_goal_grid,
                        current_planning_start_pose,
                        current_planning_goal_pose,
                        collision_waypoints=len(collision_waypoint_indices),
                        new_waypoint_cells=len(new_collision_waypoints),
                        repeated_waypoint_cells=len(repeated_collision_waypoints),
                        tracked_waypoint_cells=len(collision_waypoints_to_avoid),
                    )

                    if repeated_collision_waypoints and not new_collision_waypoints:
                        repeated_collision_samples = list(dict.fromkeys(repeated_collision_waypoints))[:3]
                        if validated_start_segment and validated_goal_segment:
                            gap_distance = np.linalg.norm(
                                np.array(current_planning_goal_grid) - np.array(current_planning_start_grid)
                            )
                            self._fail_current_goal(
                                f"❌ Path planning plateau: gap {current_planning_start_grid} → {current_planning_goal_grid} keeps reproducing the same collision waypoints\n"
                                f"   Repeated collision waypoints: {repeated_collision_samples}\n"
                                f"   Gap distance: {gap_distance:.1f} grid cells\n"
                                f"   No new obstacle cells were added in this iteration, so another A* search would repeat the same result.\n"
                                f"   Suggestion: Goal likely unreachable through this gap - try a different goal or robot starting pose."
                            )
                        else:
                            self._fail_current_goal(
                                f"❌ Path planning plateau before segments converged: repeated collision waypoints {repeated_collision_samples}\n"
                                f"   No new obstacle cells were added in this iteration, so another A* search would repeat the same result.\n"
                                f"   Suggestion: Goal likely unreachable from the current pose - try a different goal or robot starting pose."
                            )
                        return
            
            # Check if we need to continue replanning
            if waypoints_requiring_replan:
                if self.log_detailed_planning_diagnostics:
                    self.get_logger().warn(
                        f"⚠️  Detected {len(waypoints_requiring_replan)} waypoint(s) in collision. "
                        f"Tracking {len(collision_waypoints_to_avoid)} blocked waypoint cell(s) for the next iteration."
                    )
                
                replanning_attempt += 1
                if replanning_attempt >= MAX_REPLANNING_ATTEMPTS:
                    self._log_replanning_limit_hit(
                        'MAX_REPLANNING_ATTEMPTS',
                        replanning_attempt,
                        MAX_REPLANNING_ATTEMPTS,
                        'Stopping because the replanning-attempt budget was exhausted.',
                    )
                    self._fail_current_goal(
                        f"❌ Path planning FAILED after {MAX_REPLANNING_ATTEMPTS} attempts: "
                        f"{len(collision_waypoints_to_avoid)} grid cell(s) consistently have IK collision issues."
                    )
                    return
                else:
                    # Continue to next replanning iteration with updated obstacles and narrowed gap
                    self._log_hot_loop_info(lambda: f"🔄 Continuing to replanning iteration {replanning_attempt}...")
                    continue
            
            # If we reach here, all waypoints are collision-free!
            # Reconstruct complete path from validated segments
            if validated_start_segment and validated_goal_segment:
                self._log_hot_loop_info("🎉 Complete collision-free path validated!")
                
                # Build full joint trajectory from preserved segments
                all_joint_values = validated_start_segment['joint_solutions'] + validated_goal_segment['joint_solutions']
                all_joint_values_print = validated_start_segment['joint_solutions'] + validated_goal_segment['joint_solutions']
                
                # Build complete waypoint list for visualization
                all_waypoints = [wp[0] for wp in validated_start_segment['waypoints']] + [wp[0] for wp in validated_goal_segment['waypoints']]
                
                self._log_hot_loop_info(lambda: (
                    f"Final path statistics:\n"
                    f"    Start segment: {len(validated_start_segment['waypoints'])} waypoints\n"
                    f"    Goal segment:  {len(validated_goal_segment['waypoints'])} waypoints\n"
                    f"    Total:         {len(all_joint_values)} waypoints"
                ))
            else:
                # First iteration with no collisions - use all validated waypoints
                all_joint_values_print = all_joint_values
                self._log_hot_loop_info(lambda: f"🎉 Path validated with {len(all_joint_values)} collision-free waypoints!")
            
            # Break out of replanning loop - we have a valid path!
            pos = goal_pos_wrist3
            orn = goal_orientation_wrist3.as_matrix()
            T = create_pose_matrix(pos, orn)
            
            precise_goal_solutions = closed_form_algorithm(
                T,
                q_current=q_current,
                type=0,
                return_all_solutions=(self.max_ik_solutions_to_test != 1),
            )

            # Use collision checking for final goal as well
            precise_goal_joint_values, goal_selection_outcome = self._select_best_collision_free_solution(
                precise_goal_solutions,
                q_current,
                waypoint_idx="GOAL",
                test_collision=self.enable_collision_checking,
                cancel_check=lambda: self._should_abort_plan(plan_generation),
            )
            
            if precise_goal_joint_values is None:
                if goal_selection_outcome == IKSelectionOutcome.ALL_IN_COLLISION:
                    self._fail_current_goal(
                        "Goal pose has no collision-free IK solution. All solutions are in collision."
                    )
                elif goal_selection_outcome == IKSelectionOutcome.NO_VALID_SOLUTIONS:
                    self._fail_current_goal("Goal pose has no valid IK solution.")
                elif goal_selection_outcome == IKSelectionOutcome.CHECK_FAILED:
                    self._fail_current_goal("Goal pose could not be collision-validated.")
                else:
                    self._fail_current_goal(f"Unexpected IK selection outcome for goal pose: {goal_selection_outcome}")
                return
            
            # Apply unwrapping to goal solution
            delta = (precise_goal_joint_values - q_current + np.pi) % (2 * np.pi) - np.pi
            precise_goal_joint_values = q_current + delta
            for j in range(len(precise_goal_joint_values)):
                while precise_goal_joint_values[j] > 2 * np.pi:
                    precise_goal_joint_values[j] -= 2 * np.pi
                while precise_goal_joint_values[j] < -2 * np.pi:
                    precise_goal_joint_values[j] += 2 * np.pi

            all_joint_values_print.append(precise_goal_joint_values)
            all_joint_values.append(precise_goal_joint_values)
            
            # Path planning successful - break out of replanning loop
            if replanning_attempt > 0:
                self._log_hot_loop_info(lambda: (
                    f"✅ Replanning successful on attempt {replanning_attempt}: "
                    f"Found collision-free path avoiding {len(collision_waypoints_to_avoid)} problematic waypoints"
                ))
            break  # Exit replanning loop
        
        # (END OF REPLANNING LOOP - Continue with jump detection and trajectory publishing)
        if self._abort_if_requested(plan_generation, "goal was cancelled after replanning completed"):
            return

        # --- Jump detection ---
        # Maximum allowed angular change per joint between consecutive trajectory steps.
        # UR10e joint limits are [-2π, 2π]; a step larger than π/2 (90°) is considered
        # a dangerous jump and the trajectory is aborted.
        JUMP_THRESHOLD = 2 * np.pi  # radians
        jump_detected = False
        for step_i in range(1, len(all_joint_values_print)):
            delta = np.abs(np.array(all_joint_values_print[step_i], dtype=float)
                           - np.array(all_joint_values_print[step_i - 1], dtype=float))
            bad_joints = np.where(delta > JUMP_THRESHOLD)[0]
            if bad_joints.size > 0:
                self.get_logger().error(
                    f"[JUMP DETECTED] Step {step_i - 1} -> {step_i}: "
                    f"joint(s) {bad_joints.tolist()} exceeded threshold "
                    f"({JUMP_THRESHOLD:.3f} rad). "
                    f"Deltas: {np.round(delta, 3).tolist()}. "
                    f"Prev: {np.round(all_joint_values_print[step_i - 1], 3).tolist()} "
                    f"Next: {np.round(all_joint_values_print[step_i], 3).tolist()}. "
                    f"Aborting trajectory."
                )
                jump_detected = True
                break

        if jump_detected:
            with self._state_lock:
                self.execution_complete = True
            self.publish_planner_goal_failed(True)
            self._log_plan_metrics('jump_detected')
            return

        self._log_hot_loop_info(lambda: f"Joint values: {all_joint_values_print}")
        # joints_msg = Float64MultiArray()
        # flat_values = [item for sublist in all_joint_values_print for item in sublist]
        # joints_msg.data = flat_values
        # self.joint_values.publish(joints_msg)

        # Publish joint values step-by-step
        for i, q in enumerate(all_joint_values_print):
            # msg = JointState()
            # msg.name = [f'joint_{j+1}' for j in range(len(q))]
            # msg.position = q.tolist()
            # msg.header.stamp = self.get_clock().now().to_msg()
            # self.joint_pub.publish(msg)
            self._log_hot_loop_info(lambda: f"Published step {i}: {np.round(q, 3)}")
            # time.sleep(0.1)  # 10 Hz

        # Publish success
        self._log_hot_loop_info("Joint planning published.")

        if self._abort_if_requested(plan_generation, "goal was cancelled before trajectory publication"):
            return
        
        # Publish trajectory
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.expected_joint_names

        time_from_start = 1.0

        for q in all_joint_values:
            point = JointTrajectoryPoint()
            point.positions = q.tolist()
            point.velocities = [0.0] * 6  # Zero velocities = smoother interpolation
            point.time_from_start.sec = int(time_from_start)
            point.time_from_start.nanosec = int((time_from_start % 1.0) * 1e9)
            traj_msg.points.append(point)
            time_from_start += 3.0  # 3 seconds per waypoint for slow, smooth motion

        self.trajectory_pub.publish(traj_msg)
        self.publish_planner_goal_failed(False)
        self.get_logger().info("Published planned trajectory.")
        self._log_plan_metrics('success')
    
    def publish_cylinder_marker(self, center_xy, radius, z_min, z_max):
        """Publish a cylinder marker for RViz visualization."""
        marker = Marker()
        marker.header.frame_id = "arm_base"  # Change to your robot's base frame if different
        marker.header.stamp = rclpy.time.Time().to_msg()
        marker.ns = "obstacles"
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        
        # Position: center of cylinder
        marker.pose.position.x = center_xy[0]
        marker.pose.position.y = center_xy[1]
        marker.pose.position.z = (z_min + z_max) / 2.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        
        # Scale: diameter and height
        marker.scale.x = radius * 2.0  # diameter in x
        marker.scale.y = radius * 2.0  # diameter in y
        marker.scale.z = z_max - z_min  # height
        
        # Color: semi-transparent red
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.5  # semi-transparent
        
        marker.lifetime.sec = 0  # 0 means it persists until deleted
        
        self.cyl_marker_pub.publish(marker)
        self.get_logger().info(f"Published cylinder marker at ({center_xy[0]}, {center_xy[1]}) with radius {radius} and height {z_max - z_min}")

    def _create_urdf_base_mask(self, X, Y, Z, solid, pose):
        """
        Create occupancy grid mask from URDF SolidPrimitive geometry.
        
        Args:
            X, Y, Z: 3D meshgrid arrays
            solid: SolidPrimitive message with type and dimensions
            pose: Pose message with position and orientation
            
        Returns:
            Boolean mask array matching grid shape
        """
        # Extract pose
        pos = np.array([pose.position.x, pose.position.y, pose.position.z])
        quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        rot_matrix = R.from_quat(quat).as_matrix()
        
        if solid.type == SolidPrimitive.BOX:
            # Box dimensions: [x, y, z]
            size_x, size_y, size_z = solid.dimensions[0], solid.dimensions[1], solid.dimensions[2]
            half_x, half_y, half_z = size_x / 2.0, size_y / 2.0, size_z / 2.0
            
            # Transform grid points to box local frame
            # Points relative to box center
            X_rel = X - pos[0]
            Y_rel = Y - pos[1]
            Z_rel = Z - pos[2]
            
            # Rotate points to box frame (inverse rotation)
            rot_inv = rot_matrix.T
            X_local = rot_inv[0, 0] * X_rel + rot_inv[0, 1] * Y_rel + rot_inv[0, 2] * Z_rel
            Y_local = rot_inv[1, 0] * X_rel + rot_inv[1, 1] * Y_rel + rot_inv[1, 2] * Z_rel
            Z_local = rot_inv[2, 0] * X_rel + rot_inv[2, 1] * Y_rel + rot_inv[2, 2] * Z_rel
            
            # Check if inside box
            mask = (
                (np.abs(X_local) <= half_x) &
                (np.abs(Y_local) <= half_y) &
                (np.abs(Z_local) <= half_z)
            )
            return mask
            
        elif solid.type == SolidPrimitive.CYLINDER:
            # Cylinder dimensions: [height, radius]
            height, radius = solid.dimensions[0], solid.dimensions[1]
            half_height = height / 2.0
            
            # Transform points relative to cylinder center
            X_rel = X - pos[0]
            Y_rel = Y - pos[1]
            Z_rel = Z - pos[2]
            
            # Rotate to cylinder frame (cylinder axis is along local Z)
            rot_inv = rot_matrix.T
            X_local = rot_inv[0, 0] * X_rel + rot_inv[0, 1] * Y_rel + rot_inv[0, 2] * Z_rel
            Y_local = rot_inv[1, 0] * X_rel + rot_inv[1, 1] * Y_rel + rot_inv[1, 2] * Z_rel
            Z_local = rot_inv[2, 0] * X_rel + rot_inv[2, 1] * Y_rel + rot_inv[2, 2] * Z_rel
            
            # Check if inside cylinder (radial distance in XY plane, height in Z)
            radial_dist_sq = X_local**2 + Y_local**2
            mask = (radial_dist_sq <= radius**2) & (np.abs(Z_local) <= half_height)
            return mask
            
        elif solid.type == SolidPrimitive.SPHERE:
            # Sphere dimensions: [radius]
            radius = solid.dimensions[0]
            
            # Distance from sphere center
            dist_sq = (X - pos[0])**2 + (Y - pos[1])**2 + (Z - pos[2])**2
            mask = dist_sq <= radius**2
            return mask
            
        else:
            self.get_logger().error(f"Unsupported solid type for mask creation: {solid.type}")
            return np.zeros(X.shape, dtype=bool)
    
    @staticmethod
    def _point_in_polygon_mask(X, Y, polygon_xy):
        """
        Vectorised ray-casting point-in-polygon test.
        X, Y : 2-D numpy arrays of the same shape (the XY slice of the meshgrid).
        polygon_xy : list of (x, y) tuples defining the polygon vertices (in order).
        Returns a boolean array of the same shape as X.
        """
        n = len(polygon_xy)
        inside = np.zeros(X.shape, dtype=bool)
        px = np.array([v[0] for v in polygon_xy])
        py = np.array([v[1] for v in polygon_xy])
        j = n - 1
        for i in range(n):
            xi, yi = px[i], py[i]
            xj, yj = px[j], py[j]
            cond1 = (yi > Y) != (yj > Y)
            # avoid division by zero with a tiny epsilon
            x_intersect = (xj - xi) * (Y - yi) / (yj - yi + 1e-12) + xi
            cond2 = X < x_intersect
            inside ^= cond1 & cond2
            j = i
        return inside

    def publish_base_footprint_marker(self, footprint_xy, z_min, z_max):
        """
        Publish the mobile-manipulator base footprint as a 3-D prism wireframe
        (LINE_LIST) on the same 'obstacle_markers' topic as the cylinder.
        """
        now = rclpy.time.Time().to_msg()
        n = len(footprint_xy)

        marker = Marker()
        marker.header.frame_id = "arm_base"
        marker.header.stamp = now
        marker.ns = "obstacles"
        marker.id = 2  # distinct id from the cylinder (id=0) and wall (id=1)
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.015  # line width in metres

        # Colour: semi-transparent orange
        marker.color.r = 1.0
        marker.color.g = 0.5
        marker.color.b = 0.0
        marker.color.a = 0.8

        pts = marker.points
        # Bottom perimeter + top perimeter + vertical edges
        for i in range(n):
            j = (i + 1) % n
            x0, y0 = float(footprint_xy[i][0]), float(footprint_xy[i][1])
            x1, y1 = float(footprint_xy[j][0]), float(footprint_xy[j][1])

            # bottom edge
            pts.append(Point(x=x0, y=y0, z=float(z_min)))
            pts.append(Point(x=x1, y=y1, z=float(z_min)))
            # top edge
            pts.append(Point(x=x0, y=y0, z=float(z_max)))
            pts.append(Point(x=x1, y=y1, z=float(z_max)))
            # vertical edge at vertex i
            pts.append(Point(x=x0, y=y0, z=float(z_min)))
            pts.append(Point(x=x0, y=y0, z=float(z_max)))

        marker.lifetime.sec = 0  # persist until deleted

        self.cyl_marker_pub.publish(marker)
        self.get_logger().info(
            f"Published base footprint marker with {n} vertices, "
            f"z=[{z_min}, {z_max}]"
        )

def create_pose_matrix(position, rotation_matrix):
    """Helper to create 4x4 transformation matrix from position and rotation matrix."""
    T = np.eye(4)
    T[:3, :3] = rotation_matrix
    T[:3, 3] = position
    return T

def main(args=None):
    rclpy.init(args=args)
    planner_node = PlannerNode()
    try:
        rclpy.spin(planner_node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        print("Keyboard interrupt received. Shutting down planner node.")
    except Exception as e:
        print(f"Unhandled exception: {e}")

if __name__ == "__main__":
    main()
