#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import os
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, ColorRGBA
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
        self.declare_parameter('visualize_goal_orientation', True)
        self.visualize_goal_orientation = self.get_parameter('visualize_goal_orientation').get_parameter_value().bool_value

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
        self.create_subscription(PoseStamped, '/arm/goal_pose', self.goal_callback, 10)
        self.create_subscription(JointState, "joint_states", self.joint_state_callback, 10)
        self.create_subscription(Bool, "execution_status", self.execution_status_callback, 10)
        self.create_subscription(PoseStamped, "end_effector_pose", self.end_effector_pose_callback, 10)
        self.emergency_sub = self.create_subscription(Bool, "emergency_stop", self.emergency_callback, qos)
        self.trajectory_pub = self.create_publisher(JointTrajectory, 'planned_trajectory', 10)
        self.cyl_marker_pub = self.create_publisher(Marker, 'obstacle_markers', 10)
        self.wall_marker_pub = self.create_publisher(Marker, 'wall_marker', 10)
        self.goal_marker_pub = self.create_publisher(Marker, 'goal_pose_marker', 10)
        self.ee_path_marker_pub = self.create_publisher(Marker, 'ee_path_markers', 10)
        
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
        
    def publish_ee_path_markers(self, path_world, frame_id="map", current_ee_xyz=None, goal_xyz=None,
                                ns="ee_path", id_offset=0, rgba=(0.0, 0.0, 1.0, 1.0),
                                z_offset=0.0, line_width=0.01, sphere_diam=0.03):
        """Publish EE path as spheres + a line strip connecting them."""
        now = self.get_clock().now().to_msg()

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
        m.header.stamp = self.get_clock().now().to_msg()
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
        m.header.stamp = self.get_clock().now().to_msg()
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
    
    def _prune_path_endpoints(self, path_world, current_pos, goal_pos, tol_m=0.02, log=True):
        """
        Single-pass endpoint pruning + allow pruning down to empty.

        Rules:
        1) Start-neighbor rule: drop w1 if cur->w2 is significantly closer than cur->w1
        2) End-neighbor rule:   drop wn if goal->w(n-1) is significantly closer than goal->wn
        3) Progress rule start: drop w1 if w1 is farther from goal than cur is (w1 is "behind")
        4) Progress rule end:   drop wn if wn is farther from cur than goal is (wn is "beyond")
        5) If 1 waypoint remains: drop it if direct cur<->goal is better than using it from both sides
        """
        if not path_world:
            if log:
                self.get_logger().info("[prune] input path empty -> return []")
            return []

        pw = [(float(x), float(y), float(z)) for (x, y, z) in path_world]

        def d2(a, b):
            dx = a[0] - b[0]
            dy = a[1] - b[1]
            dz = a[2] - b[2]
            return dx*dx + dy*dy + dz*dz

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

        after = len(pw)
        if log:
            self.get_logger().info(f"[prune] result: {before} -> {after} waypoints")
        return pw
        
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

    def goal_callback(self, msg: PoseStamped):
        if self.emergency_stop:
            self.get_logger().warn("Emergency stop is active, aborting trajectory planning. Ignoring goal.")
            self.execution_complete = True  # Ensure we can accept new goals after emergency is cleared
            return  # Aborting if emergency state is active
        
        if not self.execution_complete:
            self.get_logger().warn("Previous trajectory not finished. Goal queued.")
            self.goal_queue.append(msg)
            return
        
        self.execution_complete = False
        pose_data = (
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        )
        self.publish_goal_marker(pose_data)
        self.plan_and_send_trajectory(msg)

    def plan_and_send_trajectory(self, msg: PoseStamped):
        if self.current_joint_state is None:
            self.get_logger().error("No current joint state received yet. Cannot calculate trajectory.")
            return
        
        # Goal definition
        goal_pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        goal_orn = [msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w]
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
        start_pos = [self.end_effector_pose.pose.position.x, self.end_effector_pose.pose.position.y, self.end_effector_pose.pose.position.z]
        start_orn = [self.end_effector_pose.pose.orientation.x, self.end_effector_pose.pose.orientation.y, self.end_effector_pose.pose.orientation.z, self.end_effector_pose.pose.orientation.w]
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
            self.execution_complete = True  # Mark execution as complete to allow new goals
            return

        # Convert path to world coordinates
        path_world_raw = [grid_to_world(i, j, k, self.x_vals, self.y_vals, self.z_vals) for i, j, k in path]
        
        # Debug: publish raw path (pre-prune)
        self.publish_ee_path_markers(
            path_world_raw,
            frame_id="arm_base",
            current_ee_xyz=start_pos,
            goal_xyz=goal_pos,
            ns="ee_path_raw",
            id_offset=100,
            rgba=(0.0, 1.0, 1.0, 0.5),   # cyan
            z_offset=0.002,              # 2mm above
            line_width=0.01,             # thicker
            sphere_diam=0.03             # bigger
        )

        # Prune path endpoints
        path_world = self._prune_path_endpoints(path_world_raw, start_pos, goal_pos, tol_m=0.02, log=True)
        path_len = len(path_world)
        if path_len == 0:
            self.get_logger().warn("Path pruned to empty. Falling back to direct goal IK.")
        
        # Publish pruned path
        self.publish_ee_path_markers(
            path_world,
            frame_id="arm_base",
            current_ee_xyz=start_pos,
            goal_xyz=goal_pos,
            ns="ee_path_pruned",
            id_offset=200,
            rgba=(1.0, 0.5, 0.0, 1.0),  # orange
            z_offset=0.0,
            line_width=0.01,
            sphere_diam=0.02
        )

        # Interpolate orientations along the (possibly pruned) path
        if path_len == 0:
            # No waypoint orientations needed; we will only use the goal orientation
            interp_rots = R.from_quat([goal_orientation.as_quat()])
            interp_rot_matrices = interp_rots.as_matrix()

        elif path_len == 1:
            # Single waypoint: rotate in place to goal orientation
            interp_rots = R.from_quat([goal_orientation.as_quat()])
            interp_rot_matrices = interp_rots.as_matrix()

        else:
            key_rots = R.from_quat([start_orientation.as_quat(), goal_orientation.as_quat()])
            slerp = Slerp([0, 1], key_rots)
            times = np.linspace(0, 1, path_len)
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
        
        # Final goal pose to ensure accuracy
        pos = goal_pos
        orn = interp_rot_matrices[-1]
        T = create_pose_matrix(pos, orn)
        
        joint_values = closed_form_algorithm(T, q_current, type=0)
        all_joint_values_print.append(joint_values)
        all_joint_values.append(joint_values)

        if np.any(np.isnan(joint_values)):
            self.get_logger().error("IK solution contains NaN. Aborting.")
            self.execution_complete = True  # Mark execution as complete to allow new goals
            return

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
            self.execution_complete = True  # Mark execution as complete to allow new goals
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
        marker.header.frame_id = "arm_base"  # Change to your robot's base frame if different
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
        
        self.cyl_marker_pub.publish(marker)
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

