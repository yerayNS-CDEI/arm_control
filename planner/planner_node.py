#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import os
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker
from ament_index_python.packages import get_package_share_directory
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

# Import planning functions
from planner.planner_lib.closed_form_algorithm import closed_form_algorithm
from planner.planner_lib.Astar3D import find_path, world_to_grid, grid_to_world, dilate_obstacles
from scipy.spatial.transform import Rotation as R, Slerp

class PlannerNode(Node):
    def __init__(self):
        super().__init__('planner_node')
        
        # Declare and get joint prefix parameter
        self.declare_parameter('joint_prefix', 'arm_')
        self.joint_prefix = self.get_parameter('joint_prefix').get_parameter_value().string_value
        
        # --- Parameters for the grid / reachability ---
        self.robot_name = 'ur10e'
        self.filename = "reachability_map_27_fused"
        fn_npy = f"{self.filename}.npy"
        self.grid_size = int(self.filename.split('_')[2])
        self.base_path = os.path.join(get_package_share_directory('arm_control'), 'resource')
        self.reachability_map_fn=os.path.join(self.base_path, fn_npy)

        self.get_logger().info(f"Loading reachability map from: {self.reachability_map_fn}")
        self.reachability_map = np.load(self.reachability_map_fn, allow_pickle=True).item()
        self.radius = 1.35  # For UR10e (hardcoded!!)

        self.execution_complete = True
        self.goal_queue = []
        self.current_joint_state = None
        self.emergency_stop = False
        self.invalid_path = False
        self.end_effector_pose = None
        self.i = 0
        
        
        # Define cilinder parameters
        self.cyl_center_xy = (0.0, 0.0)   # (xc, yc)
        self.cyl_radius    = 0.3
        self.z_min, self.z_max  = 0.0, 1.3
        
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
        self.resolution = 2*self.radius / self.grid_size
        self.x_vals = np.linspace(-self.radius + (self.resolution / 2), self.radius - (self.resolution / 2), self.grid_size)
        self.y_vals = self.x_vals
        self.z_levels = sorted(self.reachability_map.keys())
        self.grid_shape = (self.grid_size, self.grid_size, len(self.z_levels))
        z_min = min(self.z_levels)
        z_max = max(self.z_levels)
        self.z_vals = np.linspace(z_min, z_max, len(self.z_levels))  # assumes uniform spacing
        
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # Create subscriber and publishers
        self.create_subscription(Pose, 'goal_pose', self.goal_callback, 10)
        self.create_subscription(JointState, "joint_states", self.joint_state_callback, 10)
        self.create_subscription(Bool, "execution_status", self.execution_status_callback, 10)
        self.create_subscription(Pose, "end_effector_pose", self.end_effector_pose_callback, 10)
        self.emergency_sub = self.create_subscription(Bool, "emergency_stop", self.emergency_callback, qos)
        self.trajectory_pub = self.create_publisher(JointTrajectory, 'planned_trajectory', 10)
        self.marker_pub = self.create_publisher(Marker, 'obstacle_markers', 10)
        self.marker_wall_pub = self.create_publisher(Marker, 'wall_marker', 10)

        self.get_logger().info("Planner node initialized and waiting for goal poses...")
        
        #! publish simulalted wall markers
        self.publish_wall()    
        self.publish_cylinder_marker(self.cyl_center_xy, self.cyl_radius, self.z_min, self.z_max)
        
    def publish_wall(self):
        """Publish a wall marker for RViz visualization."""
        marker = Marker()
        marker.header.frame_id = "map"  # Change to your robot's base frame if different
        marker.header.stamp = self.get_clock().now().to_msg()
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
         
        self.marker_wall_pub.publish(marker)
        self.get_logger().info(f"Published wall marker at (2.5, 2.0) with thickness 0.1, width 4.0, and height 4.0")
        
    def emergency_callback(self, msg):
        self.emergency_stop = msg.data
        if self.emergency_stop:
            self.execution_complete = True

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

    def execution_status_callback(self, msg: Bool):
        self.execution_complete = msg.data
        if self.execution_complete:
            self.get_logger().info("Goal execution complete. Ready for new goal.")
            if self.goal_queue:
                next_goal = self.goal_queue.pop(0)
                self.get_logger().info("Executing next goal in queue.")
                self.execution_complete = True
                self.plan_and_send_trajectory(next_goal)

    def goal_callback(self, msg: Pose):
        if self.emergency_stop:
            self.get_logger().warn("Emergency stop is active, aborting trajectory planning. Ignoring goal.")
            return  # Aborting if emergency state is active
        
        if not self.execution_complete:
            self.get_logger().warn("Previous trajectory not finished. Goal queued.")
            self.goal_queue.append(msg)
            return
        
        self.execution_complete = False
        self.plan_and_send_trajectory(msg)

    def plan_and_send_trajectory(self, msg: Pose):
        if self.current_joint_state is None:
            self.get_logger().error("No current joint state received yet. Cannot calculate trajectory.")
            return
        
        # Goal definition
        goal_pos = [msg.position.x, msg.position.y, msg.position.z]
        goal_orn = [msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w]
        goal_orientation = R.from_quat(goal_orn)
        self.get_logger().info(f"Received goal position: {goal_pos}")
        self.get_logger().info(f"Received goal orientation: {goal_orn}")

        # Example start position (NEEDS TO be parameterized)
        # if self.i == 0:
        #     start_pos = [-0.25, 0.6, 0.2]
        #     start_orientation = R.from_euler('xyz', [60, 120, 150], degrees=True)
        #     self.i += 1
        # else:
        
        # Obtainin gcurrent robot pose from JointState
        start_pos = [self.end_effector_pose.position.x, self.end_effector_pose.position.y, self.end_effector_pose.position.z]
        start_orn = [self.end_effector_pose.orientation.x, self.end_effector_pose.orientation.y, self.end_effector_pose.orientation.z, self.end_effector_pose.orientation.w]
        start_orientation = R.from_quat(start_orn)
            
        try:
            start_idx = world_to_grid(*start_pos, self.x_vals, self.y_vals, self.z_vals)
            goal_idx = world_to_grid(*goal_pos, self.x_vals, self.y_vals, self.z_vals)
        except Exception as e:
            self.get_logger().error(f"Failed to convert world to grid: {e}")
            return

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
        
        # Publish cylinder marker for visualization
        self.publish_cylinder_marker(self.cyl_center_xy, self.cyl_radius, self.z_min, self.z_max)
        self.publish_wall()

        # Define the dilation distance (in meters)
        dilation_distance = 0.001  # Enlarge obstacles by 0.X meters in all directions

        # Apply dilation
        occupancy_grid_dilated = dilate_obstacles(occupancy_grid, dilation_distance, self.x_vals)
        # occupancy_grid_dilated = np.zeros(self.grid_shape, dtype=np.uint8)      # Eliminating all obstacles for now

        path = find_path(occupancy_grid_dilated, start_idx, goal_idx)
        self.get_logger().info(f"Found path with length {len(path)}")
        if not path:
            self.get_logger().warn("No path found!")
            return

        # Convert path to world coordinates
        path_world = [grid_to_world(i, j, k, self.x_vals, self.y_vals, self.z_vals) for i, j, k in path]
        
        # Interpolate orientations along the path
        if len(path) == 1:
            self.get_logger().info("Single-point path: rotating in place.")
            N = 1   # N steps to rotate
            if N == 1:
                interp_rots = R.from_quat([goal_orientation.as_quat()])
            else:
                interp_rots = Slerp([0, 1], R.from_quat([start_orientation.as_quat(), goal_orientation.as_quat()]))(np.linspace(0, 1, N))
            interp_rot_matrices = interp_rots.as_matrix()
        else:        
            key_rots = R.from_quat([start_orientation.as_quat(), goal_orientation.as_quat()])
            key_times = [0, 1]
            slerp = Slerp(key_times, key_rots)
            times = np.linspace(0, 1, len(path))
            interp_rots = slerp(times)
            interp_rot_matrices = interp_rots.as_matrix()

        # Convert to Euler angles
        interpolated_euler_angles = interp_rots.as_euler('xyz', degrees=True)
        print("Interpolated Euler angles:\n", interpolated_euler_angles)

        # Plan joint values
        # home_position = np.array([0.0, -1.2, -2.3, -1.2, 1.57, 0.0])
        home_position = np.array([self.current_joint_state.position])
        all_joint_values = []
        all_joint_values_print = []
        q_current = np.array([self.current_joint_state.position[i] for i in self.joint_indices])
        self.get_logger().error(f"Current joint state = {self.current_joint_state.position}")
        all_joint_values_print.append(q_current)

        for i in range(len(path_world)):
            T = create_pose_matrix(path_world[i], interp_rot_matrices[i])
            q_new = closed_form_algorithm(T, q_current, type=0)
            if np.any(np.isnan(q_new)):
                self.get_logger().error(f"Invalid IK at step {i}: pose = {T[:3, 3]}. Path world = {path_world[i]}")
                self.invalid_path = True 
            all_joint_values.append(q_new)
            all_joint_values_print.append(q_new)
            q_current = q_new

        self.get_logger().info(f"Joint values: {all_joint_values_print}")
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
            self.get_logger().info(f"Published step {i}: {np.round(q, 3)}")
            # time.sleep(0.1)  # 10 Hz

        # Publish success
        self.get_logger().info("Joint planning published.")
                
        # if self.emergency_stop:  # Verifies if emergency stop is active
        #     self.get_logger().warn("Emergency stop is active. Halting trajectory.")
        #     return
        
        if self.invalid_path:  # Verifies if invalid IK in path
            self.get_logger().warn("Invalid path detected. Halting trajectory.")
            self.invalid_path = False
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
        self.get_logger().info("Published planned trajectory.")
    
    def publish_cylinder_marker(self, center_xy, radius, z_min, z_max):
        """Publish a cylinder marker for RViz visualization."""
        marker = Marker()
        marker.header.frame_id = "arm_base_link"  # Change to your robot's base frame if different
        marker.header.stamp = self.get_clock().now().to_msg()
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
        
        self.marker_pub.publish(marker)
        self.get_logger().info(f"Published cylinder marker at ({center_xy[0]}, {center_xy[1]}) with radius {radius} and height {z_max - z_min}")

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

