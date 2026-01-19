#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Bool
from arm_control.srv import ComputeWallDiscretization
from statistics import median
from typing import List, Tuple
from collections import deque
import numpy as np


class ExhaustiveScanNode(Node):
    def __init__(self):
        super().__init__('exhaustive_scan_node')
        
        # Publisher for goal poses
        self.goal_pub = self.create_publisher(Pose, 'goal_pose', 10)
        
        # Subscriber for execution status
        self.execution_status_sub = self.create_subscription(
            Bool,
            'execution_status',
            self.execution_status_callback,
            10
        )
        
        # Service client for wall discretization
        self.discretization_client = self.create_client(
            ComputeWallDiscretization, 
            'compute_wall_discretization'
        )
        
        # State variables
        self.goals_queue = deque()
        self.current_goal_idx = -1
        self.waiting_for_arrival = False
        self.scan_completed = False
        self.execution_status = False
        
        # Configuration parameters
        self.declare_parameter('robot_amplitude_range', 1.3)
        self.declare_parameter('robot_height_range', 1.3)
        self.declare_parameter('sensors_amplitude_range', 0.4)
        self.declare_parameter('sensors_height_range', 0.4)
        self.declare_parameter('resolution', 0.1)
        self.declare_parameter('scan_orientation_x', -0.5)
        self.declare_parameter('scan_orientation_y', -0.5)
        self.declare_parameter('scan_orientation_z', -0.5)
        self.declare_parameter('scan_orientation_w', -0.5)
        
        self.get_logger().info("Exhaustive Scan Node initialized.")
        
    def execution_status_callback(self, msg):
        """Callback for execution status updates."""
        self.execution_status = msg.data
        if self.execution_status and self.waiting_for_arrival:
            self.waiting_for_arrival = False
            self.get_logger().info(f"✓ Goal {self.current_goal_idx + 1} reached.")
            
    def _estimate_step_threshold(self, sorted_vals: List[float], resolution: float = None) -> float:
        """Estimate threshold to separate rows based on consecutive value differences."""
        if len(sorted_vals) < 2:
            return float('inf')
        diffs = [abs(sorted_vals[i+1] - sorted_vals[i]) for i in range(len(sorted_vals)-1)]
        if resolution and resolution > 0:
            return 0.6 * float(resolution)
        m = median(diffs) if diffs else 0.0
        return max(1e-6, 1.5 * m)
    
    def serpentine_order_vertical_z(self, current_position: Point, 
                                   panel_cells_centers: List[Pose], 
                                   resolution: float = None) -> List[Pose]:
        """
        Order poses in serpentine pattern assuming:
        - v = Z (vertical, rows: from higher Z to lower Z → "top" first)
        - u = projection of (X,Y) onto main wall direction (2D PCA)
        Returns list of Poses in S order: row1 left→right, row2 right→left, ...
        """
        if not panel_cells_centers:
            return []

        X = np.array([p.position.x for p in panel_cells_centers], dtype=float)
        Y = np.array([p.position.y for p in panel_cells_centers], dtype=float)
        Z = np.array([p.position.z for p in panel_cells_centers], dtype=float)
        z_min = min(Z)

        # 2D PCA on (X,Y) to find main direction û
        XY = np.vstack([X, Y])  # shape (2, N)
        XY_centered = XY - XY.mean(axis=1, keepdims=True)
        cov = np.cov(XY_centered)
        eigvals, eigvecs = np.linalg.eigh(cov)
        u_hat = eigvecs[:, np.argmax(eigvals)]  # unit vector in XY
        u_hat = u_hat / np.linalg.norm(u_hat)

        # Scalar projection u = <(X,Y), û>
        U = XY_centered.T @ u_hat  # shape (N,)

        # Preliminary order by descending Z (top row first)
        idx = np.argsort(-Z)
        U_sorted = U[idx]
        Z_sorted = Z[idx]
        
        # Adjust coordinates relative to current position
        poses_sorted = []
        for i in idx:
            pose = Pose()
            pose.position.x = panel_cells_centers[i].position.x - current_position.x
            pose.position.y = panel_cells_centers[i].position.y - current_position.y
            pose.position.z = panel_cells_centers[i].position.z - z_min
            pose.orientation = panel_cells_centers[i].orientation
            poses_sorted.append(pose)

        # Estimate threshold to separate rows by Z
        thr = self._estimate_step_threshold(list(Z_sorted), resolution=resolution)

        # Partition into rows (by gaps in Z)
        rows_idx = [[0]]
        for i in range(1, len(Z_sorted)):
            if abs(Z_sorted[i] - Z_sorted[i-1]) > thr:
                rows_idx.append([i])
            else:
                rows_idx[-1].append(i)

        # Serpentine order: per row, sort by U ascending and alternate direction
        ordered = []
        for r, row in enumerate(rows_idx):
            row_items = [(U_sorted[i], poses_sorted[i]) for i in row]
            row_items.sort(key=lambda t: t[0])  # left→right by ascending u
            if r % 2 == 1:
                row_items.reverse()  # serpentine: odd rows reverse
            ordered.extend([p for _, p in row_items])

        return ordered
    
    def compute_wall_discretization(self, start_point: Point, end_point: Point) -> bool:
        """Call the wall discretization service and generate scan trajectory."""
        self.get_logger().info("Waiting for wall discretization service...")
        if not self.discretization_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Wall discretization service not available.")
            return False
        
        # Build service request
        req = ComputeWallDiscretization.Request()
        req.start_point = start_point
        req.end_point = end_point
        req.robot_amplitude_range = self.get_parameter('robot_amplitude_range').value
        req.robot_height_range = self.get_parameter('robot_height_range').value
        req.sensors_amplitude_range = self.get_parameter('sensors_amplitude_range').value
        req.sensors_height_range = self.get_parameter('sensors_height_range').value
        req.resolution = self.get_parameter('resolution').value
        req.target_i = 1
        req.target_j = 1
        
        self.get_logger().info("Calling wall discretization service...")
        future = self.discretization_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if not future.done():
            self.get_logger().error("Wall discretization service call timed out.")
            return False
        
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error(f"Wall discretization service call failed: {e}")
            return False
        
        if not result.success:
            self.get_logger().error("Wall discretization service returned failure.")
            return False
        
        self.get_logger().info(f"Wall discretization successful. Received {len(result.panel_cells_centers)} scan points.")
        
        # Generate serpentine trajectory
        current_position = Point()
        current_position.x = start_point.x
        current_position.y = start_point.y
        current_position.z = start_point.z
        
        resolution = self.get_parameter('resolution').value
        ordered_goals = self.serpentine_order_vertical_z(
            current_position, 
            result.panel_cells_centers, 
            resolution=resolution
        )
        
        # Set scan orientation for all goals
        scan_qx = self.get_parameter('scan_orientation_x').value
        scan_qy = self.get_parameter('scan_orientation_y').value
        scan_qz = self.get_parameter('scan_orientation_z').value
        scan_qw = self.get_parameter('scan_orientation_w').value
        
        for pose in ordered_goals:
            pose.orientation.x = scan_qx
            pose.orientation.y = scan_qy
            pose.orientation.z = scan_qz
            pose.orientation.w = scan_qw
            self.goals_queue.append(pose)
        
        self.get_logger().info(f"Generated serpentine trajectory with {len(self.goals_queue)} waypoints.")
        return True
    
    def start_scan(self, start_point: Point, end_point: Point) -> bool:
        """Initialize and start the exhaustive scan."""
        self.goals_queue.clear()
        self.current_goal_idx = -1
        self.waiting_for_arrival = False
        self.scan_completed = False
        self.execution_status = False
        
        if not self.compute_wall_discretization(start_point, end_point):
            return False
        
        self.get_logger().info("Starting exhaustive scan...")
        return True
    
    def send_next_goal(self) -> bool:
        """Send the next goal from the queue."""
        if not self.goals_queue:
            self.scan_completed = True
            self.get_logger().info("✓ All scan goals completed!")
            return False
        
        next_goal = self.goals_queue.popleft()
        self.current_goal_idx += 1
        
        self.execution_status = False
        self.goal_pub.publish(next_goal)
        self.waiting_for_arrival = True
        
        remaining = len(self.goals_queue)
        self.get_logger().info(
            f"→ Sent goal {self.current_goal_idx + 1} (remaining: {remaining})\n"
            f"   pos=({next_goal.position.x:.3f}, {next_goal.position.y:.3f}, {next_goal.position.z:.3f})\n"
            f"   orn=({next_goal.orientation.x:.3f}, {next_goal.orientation.y:.3f}, "
            f"{next_goal.orientation.z:.3f}, {next_goal.orientation.w:.3f})"
        )
        return True
    
    def spin_scan(self):
        """Main loop to execute the scan trajectory."""
        rate = self.create_rate(10)  # 10 Hz
        
        while rclpy.ok() and not self.scan_completed:
            if not self.waiting_for_arrival:
                if not self.send_next_goal():
                    break
            
            rclpy.spin_once(self, timeout_sec=0.1)
            
            try:
                rate.sleep()
            except:
                break
        
        if self.scan_completed:
            self.get_logger().info("Exhaustive scan completed successfully!")
        else:
            self.get_logger().warn("Exhaustive scan interrupted.")


def main(args=None):
    rclpy.init(args=args)
    
    node = ExhaustiveScanNode()
    
    import sys
    if len(sys.argv) < 7:
        node.get_logger().info(
            "Usage: ros2 run arm_control exhaustive_scan_node "
            "<start_x> <start_y> <start_z> <end_x> <end_y> <end_z>"
        )
        node.get_logger().info(
            "Example: ros2 run arm_control exhaustive_scan_node 0.0 0.0 0.0 2.0 0.0 2.0"
        )
        node.destroy_node()
        rclpy.shutdown()
        return
    
    try:
        start_point = Point()
        start_point.x = float(sys.argv[1])
        start_point.y = float(sys.argv[2])
        start_point.z = float(sys.argv[3])
        
        end_point = Point()
        end_point.x = float(sys.argv[4])
        end_point.y = float(sys.argv[5])
        end_point.z = float(sys.argv[6])
        
        node.get_logger().info(f"Start point: ({start_point.x}, {start_point.y}, {start_point.z})")
        node.get_logger().info(f"End point: ({end_point.x}, {end_point.y}, {end_point.z})")
        
        if node.start_scan(start_point, end_point):
            node.spin_scan()
        else:
            node.get_logger().error("Failed to start scan.")
    
    except ValueError as e:
        node.get_logger().error(f"Invalid coordinate values: {e}")
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user.")
    except Exception as e:
        node.get_logger().error(f"Error: {e}")
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
