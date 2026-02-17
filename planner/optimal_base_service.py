#!/usr/bin/env python3

import os
import numpy as np
import time
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from scipy.spatial.transform import Rotation as R

from arm_control.srv import OptimalBase
from planner.rm4d_lib.reachability_map import ReachabilityMap4D
from planner.rm4d_lib.base_pos_grid import BasePosGrid
from planner.rm4d_lib.robots import Simulator, UR10E
from planner.planner_lib.closed_form_algorithm import closed_form_algorithm

# from rclpy.executors import MultiThreadedExecutor 

class OptimalBaseService(Node):
    def __init__(self):
        super().__init__('optimal_base_service')
        # Parámetro para localizar el mapa (relativo al share o absoluto)
        self.declare_parameter('map_relpath', 'resource/rmap.npy')

        # Cargar mapa
        pkg_share = get_package_share_directory('arm_control')
        map_rel = self.get_parameter('map_relpath').value
        map_path = map_rel if os.path.isabs(map_rel) else os.path.join(pkg_share, map_rel)
        self.rmap = ReachabilityMap4D.from_file(map_path)
        self.get_logger().info(f"Mapa cargado una vez: {map_path} shape={self.rmap.map.shape}")

        # TIP: protege contra escrituras accidentales
        try:
            self.rmap.map.setflags(write=False)  # numpy: solo-lectura
        except Exception:
            pass
    
        # Servicio
        self.srv = self.create_service(OptimalBase, 'compute_optimal_base', self.handle_request)
        self.get_logger().info("Service already running.")

        # Sim opcional como atributo para mantener la ventana viva si se pide
        self.sim = None

    # Helpers
    def _poses_from_flat(self, arr):
        if len(arr) == 0:
            # default de 4 poses de tu ejemplo
            pose1 = np.eye(4); pose1[:3, 3] = [1.7, 1.6, 0.8]
            pose2 = np.eye(4); pose2[:3, 3] = [1.7, 2.0, 0.8]
            pose3 = np.eye(4); pose3[:3, 3] = [1.7, 1.6, 0.3]
            pose4 = np.eye(4); pose4[:3, 3] = [1.7, 2.0, 0.3]
            rot = R.from_euler('xyz', [0.0, 90.0, 0.0], degrees=True).as_matrix()
            for P in (pose1, pose2, pose3, pose4):
                P[:3, :3] = rot @ P[:3, :3]
            return np.stack([pose1, pose2, pose3, pose4])
        if len(arr) % 6 != 0:
            raise ValueError("poses_ee_xyzrpy debe ser múltiplo de 6")
        mats = []
        for k in range(0, len(arr), 6):
            x, y, z, roll, pitch, yaw = arr[k:k+6]
            T = np.eye(4)
            T[:3, 3] = [x, y, z]
            T[:3, :3] = R.from_euler('xyz', [roll, pitch, yaw], degrees=True).as_matrix() @ T[:3, :3]
            mats.append(T)
        return np.stack(mats)

    def _world_to_idx(self, x, y, x_limits, y_limits, n_bins_x, n_bins_y):
        res_x = (x_limits[1] - x_limits[0]) / n_bins_x
        res_y = (y_limits[1] - y_limits[0]) / n_bins_y
        i = int((x - x_limits[0]) / res_x)
        j = int((y - y_limits[0]) / res_y)
        i = min(max(i, 0), n_bins_x - 1)
        j = min(max(j, 0), n_bins_y - 1)
        return i, j

    def _apply_rect_obstacles(self, grid, rects, x_limits, y_limits):
        if not rects:
            return
        n_bins_x, n_bins_y = grid.shape
        for (x0, x1, y0, y1) in rects:
            xa, xb = sorted([x0, x1])
            ya, yb = sorted([y0, y1])
            i0, j0 = self._world_to_idx(xa, ya, x_limits, y_limits, n_bins_x, n_bins_y)
            i1, j1 = self._world_to_idx(xb, yb, x_limits, y_limits, n_bins_x, n_bins_y)
            i0, i1 = min(i0, i1), max(i0, i1)
            j0, j1 = min(j0, j1), max(j0, j1)
            grid[i0:i1+1, j0:j1+1] = 0

    def _apply_circle_obstacles(self, grid, circles, x_limits, y_limits):
        if not circles:
            return
        n_bins_x, n_bins_y = grid.shape
        xs = np.linspace(x_limits[0] + 0.5*(x_limits[1]-x_limits[0])/n_bins_x,
                         x_limits[1] - 0.5*(x_limits[1]-x_limits[0])/n_bins_x, n_bins_x)
        ys = np.linspace(y_limits[0] + 0.5*(y_limits[1]-y_limits[0])/n_bins_y,
                         y_limits[1] - 0.5*(y_limits[1]-y_limits[0])/n_bins_y, n_bins_y)
        XX, YY = np.meshgrid(xs, ys, indexing='ij')
        for (xc, yc, r) in circles:
            mask = ((XX - xc)**2 + (YY - yc)**2) <= (r**2)
            grid[mask] = 0

    def handle_request(self, req, res):
        self.get_logger().info("Request received.")
        try:
            # 2) Poses, límites y grid
            poses_ee = self._poses_from_flat(list(req.poses_ee_xyzrpy))
            if len(req.x_limits) != 2 or len(req.y_limits) != 2:
                raise ValueError("x_limits e y_limits deben tener longitud 2.")

            x_limits = list(req.x_limits)
            y_limits = list(req.y_limits)
            res_m = float(req.grid_res)
            n_bins_x = int((x_limits[1] - x_limits[0]) / res_m)
            n_bins_y = int((y_limits[1] - y_limits[0]) / res_m)

            # 3) Grids individuales + intersección
            grids = []
            for pose in poses_ee:
                g = BasePosGrid(x_limits=x_limits, y_limits=y_limits,
                                n_bins_x=n_bins_x, n_bins_y=n_bins_y)
                base_pos = self.rmap.get_base_positions(pose)
                g.add_base_positions(base_pos)
                grids.append(g)
            from copy import deepcopy
            gi = deepcopy(grids)
            for i in range(1, len(gi)):
                gi[0].intersect(gi[i])

            # 4) Redondeo
            dec = int(req.round_decimals)
            gi[0].grid = np.round(gi[0].grid, dec)

            # 5) Obstáculos (rects + circles)
            rects = []
            if len(req.obstacle_rects) % 4 != 0:
                raise ValueError("obstacle_rects debe ser múltiplo de 4.")
            for k in range(0, len(req.obstacle_rects), 4):
                if k+3 < len(req.obstacle_rects):
                    rects.append(list(req.obstacle_rects[k:k+4]))

            circles = []
            if len(req.obstacle_circles) % 3 != 0:
                raise ValueError("obstacle_circles debe ser múltiplo de 3.")
            for k in range(0, len(req.obstacle_circles), 3):
                if k+2 < len(req.obstacle_circles):
                    circles.append(list(req.obstacle_circles[k:k+3]))

            self._apply_rect_obstacles(gi[0].grid, rects, x_limits, y_limits)
            self._apply_circle_obstacles(gi[0].grid, circles, x_limits, y_limits)

            if not np.any(gi[0].grid):
                res.success = False
                res.message = "No hay posiciones válidas tras aplicar obstáculos."
                return res

            # 6) Selección: óptimos → min_dist → detrás de goals según orientación
            G = gi[0].grid
            max_val = float(np.max(G))
            max_idx = np.argwhere(G == max_val)
            cands_xy_all = [gi[0].idx_to_xy(int(i), int(j)) for (i, j) in max_idx]

            xs = poses_ee[:, 0, 3]; ys = poses_ee[:, 1, 3]
            cx, cy = float(np.mean(xs)), float(np.mean(ys))
            min_dist = float(req.min_dist)

            # Fit a plane through the goal positions (XY projection)
            # We'll use PCA to find the plane's normal direction
            goal_points_xy = np.column_stack([xs, ys])  # Nx2 array
            centroid_2d = np.array([cx, cy])
            
            # Center the points
            centered_points = goal_points_xy - centroid_2d
            
            # Compute covariance and find principal components
            if len(centered_points) > 1:
                cov = np.cov(centered_points.T)
                eigenvalues, eigenvectors = np.linalg.eig(cov)
                # The normal to the plane is the eigenvector with smallest eigenvalue
                # But for 2D, we can also use the perpendicular to the main direction
                min_eig_idx = np.argmin(eigenvalues)
                plane_normal = eigenvectors[:, min_eig_idx]
                # Ensure it's normalized
                plane_normal = plane_normal / (np.linalg.norm(plane_normal) + 1e-9)
            else:
                # Single goal or all goals at same position - no plane, use default
                plane_normal = None

            def get_perpendicular_dist_to_plane(x, y):
                """Returns perpendicular distance from (x,y) to the plane of goals"""
                if plane_normal is None:
                    # Fallback to point distance
                    return np.min(np.hypot(x - xs, y - ys))
                # Vector from centroid to candidate
                vec = np.array([x - cx, y - cy])
                # Perpendicular distance is the projection onto the plane normal
                perp_dist = abs(np.dot(vec, plane_normal))
                return perp_dist

            def ok_min_dist(x, y):
                return get_perpendicular_dist_to_plane(x, y) >= min_dist

            # Filter candidates that meet minimum perpendicular distance requirement
            cands_xy = [(x, y) for (x, y) in cands_xy_all if ok_min_dist(x, y)]
            
            if not cands_xy:
                self.get_logger().warn(f"No candidates meet min_dist={min_dist}m perpendicular to goals plane. Using closest available candidates.")
                # Fallback: use candidates closest to meeting the requirement
                cands_with_dist = [(xy, get_perpendicular_dist_to_plane(xy[0], xy[1])) for xy in cands_xy_all]
                cands_with_dist.sort(key=lambda x: -x[1])  # Sort by distance descending
                cands_xy = [xy for xy, _ in cands_with_dist[:min(20, len(cands_with_dist))]]
            
            self.get_logger().info(f"Plane normal: {plane_normal}, centroid: ({cx:.2f}, {cy:.2f})")

            # Compute "behind" direction based on goal orientations
            # Extract forward direction (Z-axis) from each pose's rotation matrix
            forward_vectors = []
            for pose in poses_ee:
                # Z-axis (forward direction) is the 3rd column of rotation matrix
                forward_z = pose[:3, 2]
                # Project to XY plane and normalize
                forward_xy = np.array([forward_z[0], forward_z[1]])
                if np.linalg.norm(forward_xy) > 1e-6:
                    forward_xy = forward_xy / np.linalg.norm(forward_xy)
                    forward_vectors.append(forward_xy)
            
            # Compute average "behind" direction (opposite of forward)
            if forward_vectors:
                avg_forward = np.mean(forward_vectors, axis=0)
                avg_forward = avg_forward / (np.linalg.norm(avg_forward) + 1e-9)
                behind_direction = -avg_forward  # Behind is opposite of forward
                
                # Score each candidate: balance "behind" alignment with proximity to goals
                def combined_score(x, y):
                    # Vector from centroid to candidate
                    vec_to_cand = np.array([x - cx, y - cy])
                    dist_to_centroid = np.linalg.norm(vec_to_cand)
                    
                    # Alignment score: prefer positions "behind" the goals
                    alignment = 0.0
                    if dist_to_centroid > 1e-6:
                        vec_to_cand_norm = vec_to_cand / dist_to_centroid
                        # Dot product: positive when candidate is in "behind" direction
                        alignment = np.dot(vec_to_cand_norm, behind_direction)
                        # Only consider candidates with positive alignment (actually behind)
                        alignment = max(0.0, alignment)
                    
                    # Distance score: prefer positions closer to centroid but respect min_dist
                    # Use perpendicular distance to the goals plane for safety
                    perp_dist = get_perpendicular_dist_to_plane(x, y)
                    # Penalize positions too close to min_dist, prefer slightly farther
                    safety_margin = 0.2  # prefer at least 0.2m beyond min_dist
                    distance_score = 1.0 / (dist_to_centroid + 0.3)  # Closer to centroid is better
                    
                    # Combined score: balance alignment and proximity
                    # Higher weight on distance to keep base closer
                    score = alignment * 0.3 + distance_score * 0.7
                    
                    # Apply penalty if too close to min_dist perpendicular to plane (safety margin violation)
                    if perp_dist < min_dist + safety_margin:
                        penalty = (min_dist + safety_margin - perp_dist) / safety_margin
                        score *= (1.0 - 0.5 * penalty)  # Up to 50% penalty
                    
                    return score
                
                # Select candidate with best combined score
                x_sel, y_sel = max(cands_xy, key=lambda xy: combined_score(xy[0], xy[1]))
                self.get_logger().info(f"Base positioned behind goals with proximity: forward={avg_forward}, behind={behind_direction}")
            else:
                # Fallback: use centroid-based selection
                x_sel, y_sel = min(cands_xy, key=lambda xy: np.hypot(xy[0]-cx, xy[1]-cy))
                self.get_logger().info("No valid orientation vectors, using centroid-based selection")

            # 7) (Opcional) Visualización si se pide en la request
            if req.enable_simulator:
                try:
                    self.sim = Simulator(with_gui=True)
                    gi[0].visualize_in_sim(self.sim)
                    self.sim.add_frame([0,0,0], [0,0,0,1])
                    for i, tf in enumerate(poses_ee):
                        pos, quat = self.sim.tf_to_pos_quat(tf)
                        self.sim.add_frame(pos, quat)
                        self.sim.add_sphere(pos=pos, radius=0.03, color=[0.0, 0.5, 1.0])  # soft blue

                    for (vx, vy) in cands_xy[:20]:
                        self.sim.add_sphere(pos=[vx, vy, 0.02], radius=0.03, color=[0.2, 1.0, 0.2])

                    robot_vis = None
                    if req.enable_robot_viz:
                        robot_vis = UR10E(self.sim, base_pos=[x_sel, y_sel, 0], base_orn=[0,0,1,0])

                    # Only run joint playback if the robot actually loaded
                    if robot_vis is not None:
                        home_position = np.array([0.0, -1.2, -2.3, -1.2, 1.57, 0.0])
                        poses_ee_translated = poses_ee.copy()
                        poses_ee_translated[:, 0, 3] -= x_sel
                        poses_ee_translated[:, 1, 3] -= y_sel
                        for i in range(len(poses_ee)):
                            modified_pose = poses_ee_translated[i, :, :].copy()
                            q_current = closed_form_algorithm(modified_pose, np.array(home_position), type=0)
                            robot_vis.reset_joint_pos(q_current)
                            self.get_logger().info(f"Step {i}: q = {np.round(q_current, 4)}")
                            time.sleep(2)
                except Exception as e:
                    self.get_logger().warn(f"Visualization failed (continuing without viz): {e}")


            self.get_logger().info("Optimal base computed succesfully.")
            # 8) Rellenar respuesta
            res.success = True
            res.message = "Optimal base computed succesfully"
            res.base_x = float(x_sel)
            res.base_y = float(y_sel)
            res.centroid_x = cx
            res.centroid_y = cy
            res.max_grid_value = max_val

            flat_cands = []
            for (x, y) in cands_xy[:40]:  # limita tamaño de respuesta
                flat_cands += [float(x), float(y)]
            res.candidates_xy = flat_cands
            return res

        except Exception as e:
            self.get_logger().error(f"Computation failed.  Error: {e}")
            res.success = False
            res.message = f"Computation failed. Error: {e}"
            return res

def main(args=None):
    rclpy.init(args=args)
    node = OptimalBaseService()

    # Monohilo
    rclpy.spin(node)

    # Paralelismo (peticiones concurrentes):
    # exec = MultiThreadedExecutor(num_threads=4)
    # exec.add_node(node)
    # exec.spin()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()