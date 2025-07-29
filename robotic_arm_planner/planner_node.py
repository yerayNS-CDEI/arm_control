import rclpy
from rclpy.node import Node
import os
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray

import numpy as np

# Import your planning functions
from robotic_arm_planner.planner_lib.closed_form_algorithm import closed_form_algorithm
from robotic_arm_planner.planner_lib.Astar3D import find_path, world_to_grid, grid_to_world, dilate_obstacles
from scipy.spatial.transform import Rotation as R, Slerp

class PlannerNode(Node):
    def __init__(self):
        super().__init__('planner_node')
        
        # --- Parameters for the grid / reachability ---
        self.robot_name = 'ur10e'
        self.filename = "reachability_map_27_fused"
        fn_npy = f"{self.filename}.npy"
        self.grid_size = int(self.filename.split('_')[2])
        base_home = os.path.expanduser('~')
        self.reachability_map_fn = os.path.join(
            base_home,
            'ws_Robotic_Arm',
            'src',
            'robotic_arm_planner',
            'resource',
            fn_npy
        )
        self.get_logger().info(f"Loading reachability map from: {self.reachability_map_fn}")
        self.reachability_map = np.load(self.reachability_map_fn, allow_pickle=True).item()
        self.radius = 1.35  # For UR10e (hardcoded!!)

        self.execution_complete = True
        self.goal_queue = []
        self.current_joint_state = None
        self.emergency_stop = False
        self.end_effector_pose = None
        self.i = 0

        # Create grid space
        self.resolution = 2*self.radius / self.grid_size
        self.x_vals = np.linspace(-self.radius + (self.resolution / 2), self.radius - (self.resolution / 2), self.grid_size)
        self.y_vals = self.x_vals
        self.z_levels = sorted(self.reachability_map.keys())
        self.grid_shape = (self.grid_size, self.grid_size, len(self.z_levels))
        z_min = min(self.z_levels)
        z_max = max(self.z_levels)
        self.z_vals = np.linspace(z_min, z_max, len(self.z_levels))  # assumes uniform spacing

        # Create subscriber and publishers
        self.create_subscription(Pose, '/goal_pose', self.goal_callback, 10)
        self.create_subscription(JointState, "/joint_states", self.joint_state_callback, 10)
        self.create_subscription(Bool, "/execution_status", self.execution_status_callback, 10)
        self.create_subscription(Pose, "/end_effector_pose", self.end_effector_pose_callback, 10)
        self.trajectory_pub = self.create_publisher(JointTrajectory, '/planned_trajectory', 10)

        self.get_logger().info("Planner node initialized and waiting for goal poses...")

    def joint_state_callback(self, msg):
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
        if not self.execution_complete:
            self.get_logger().warn("Previous trajectory not finished. Goal queued.")
            self.goal_queue.append(msg)
            return

        self.execution_complete = False
        self.plan_and_send_trajectory(msg)

    def plan_and_send_trajectory(self, msg: Pose):
        if self.emergency_stop:
            self.get_logger().warn("Emergency stop is active, aborting trajectory planning.")
            return  # Aborting if emergency state is active

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
        occupancy_grid[cube_mask] = 1

        # Define sphere parameters
        sphere_center = [(-0.3, 0.0, 0.4), (0.3, 0.5, 0.4)]
        sphere_radius = [0.3, 0.3]

        # Create mask for sphere
        for i in range(len(sphere_radius)):
            dist_squared = (X - sphere_center[i][0])**2 + (Y - sphere_center[i][1])**2 + (Z - sphere_center[i][2])**2
            sphere_mask = dist_squared <= sphere_radius[i]**2
            occupancy_grid[sphere_mask] = 1

        # Define the dilation distance (in meters)
        dilation_distance = 0.4  # Enlarge obstacles by 0.X meters in all directions

        # Apply dilation
        occupancy_grid_dilated = dilate_obstacles(occupancy_grid, dilation_distance, self.x_vals)
        occupancy_grid_dilated = np.zeros(self.grid_shape, dtype=np.uint8)      # Eliminating all obstacles for now

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
        q_current = np.array([self.current_joint_state.position[-1], self.current_joint_state.position[0], self.current_joint_state.position[1], self.current_joint_state.position[2], self.current_joint_state.position[3], self.current_joint_state.position[4]])
        self.get_logger().error(f"Current joint state = {self.current_joint_state.position}")
        all_joint_values_print.append(q_current)

        for i in range(len(path_world)):
            T = create_pose_matrix(path_world[i], interp_rot_matrices[i])
            q_new = closed_form_algorithm(T, q_current, type=0)
            if np.any(np.isnan(q_new)):
                self.get_logger().error(f"Invalid IK at step {i}: pose = {T[:3, 3]}. Path wolrd = {path_world[i]}")
                self.emergency_stop = True 
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
                
        if self.emergency_stop:  # Verifies if emergency stop is active
            self.get_logger().warn("Emergency stop is active. Halting trajectory.")
            return
        
        # Publish trajectory
        traj_msg = JointTrajectory()
        traj_msg.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]

        time_from_start = 1.0

        for q in all_joint_values:
            point = JointTrajectoryPoint()
            point.positions = q.tolist()
            point.time_from_start.sec = int(time_from_start)
            point.time_from_start.nanosec = int((time_from_start % 1.0) * 1e9)
            traj_msg.points.append(point)
            time_from_start += 1

        self.trajectory_pub.publish(traj_msg)
        self.get_logger().info("Published planned trajectory.")

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

