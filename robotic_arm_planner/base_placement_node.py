import os
import sys
import rclpy
from rclpy.node import Node
import numpy as np
import pickle
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from robotic_arm_planner_interfaces.srv import ComputeBasePlacement
from geometry_msgs.msg import Pose


class BasePlacementNode(Node):
    def __init__(self):
        super().__init__('base_placement_node')

        # Define parameters (in case they need to be modified)
        self.declare_parameter('cart_step', 0.05)
        self.declare_parameter('n_orientations', 20)
        self.declare_parameter('area_size', 1.6 * 2)
        self.declare_parameter('global_size', 4.0)
        self.declare_parameter('cart_min', -1.6)

        self.cart_step = self.get_parameter('cart_step').value
        self.global_size = self.get_parameter('global_size').value
        self.area_size = self.get_parameter('area_size').value
        self.cart_min = self.get_parameter('cart_min').value

        self.base_path = os.path.join(os.path.expanduser('~'), 'ws_reachability', 'rm4d', 'experiment_scripts')

        self.get_logger().info("Base placement node ready.")

        inp = input("Be advised that the following process loads two databases into the computer's memory."
                    "This process can be resource consuming and crash the computer. Do you want to proceed? [y/n]").strip()
        if inp != 'y':
            self.get_logger().info("Aborting process. Node will not start.")
            rclpy.shutdown()
            sys.exit(0)

        # Load database
        self.db, self.orientations = self.load_database(
            db_path=os.path.join(self.base_path, f"voxels_data_{self.get_parameter('cart_step').value}_step_{self.get_parameter('n_orientations').value}_orientations.pkl"),
            orientations_path=os.path.join(self.base_path, f"orientations_{self.get_parameter('n_orientations').value}.pkl")
        )
        # self.get_logger().info(f"Some db keys: {list(self.db.keys())[:5]}")
        # self.get_logger().info(f"Example db content: {self.db[list(self.db.keys())[0]]}")

        # Configuring service
        self.srv = self.create_service(ComputeBasePlacement, 'compute_base_placement', self.computeBasePlacementCallback)
        self.get_logger().info("Service 'compute_base_placement' ready.")
        
        # # Example objectives (this node can receive inputs from other nodes)
        # self.example_targets = [
        #     (np.array([1.7, 1.6, 0.8]), R.from_euler('xyz', [0, 90, 0], degrees=True).as_matrix()),
        #     (np.array([1.7, 2.0, 0.8]), R.from_euler('xyz', [0, 90, 0], degrees=True).as_matrix()),
        #     (np.array([1.7, 1.6, 0.3]), R.from_euler('xyz', [0, 90, 0], degrees=True).as_matrix()),
        #     (np.array([1.7, 2.0, 0.3]), R.from_euler('xyz', [0, 90, 0], degrees=True).as_matrix())
        # ]

        # ros2 service call /compute_base_placement robotic_arm_planner_interfaces/srv/ComputeBasePlacement "targets:
        # - position: {x: 1.7, y: 1.6, z: 0.8}
        # orientation: {x: 0.0, y: 0.7071, z: 0.0, w: 0.7071}
        # - position: {x: 1.7, y: 2.0, z: 0.8}
        # orientation: {x: 0.0, y: 0.7071, z: 0.0, w: 0.7071}
        # - position: {x: 1.7, y: 1.6, z: 0.3}
        # orientation: {x: 0.0, y: 0.7071, z: 0.0, w: 0.7071}
        # - position: {x: 1.7, y: 2.0, z: 0.3}
        # orientation: {x: 0.0, y: 0.7071, z: 0.0, w: 0.7071}"

    def cleanup(self):
        """Cleanup function to free memory from the databases loaded."""
        self.get_logger().info("Cleaning up resources...")
        if hasattr(self, 'db'):
            self.db.clear()
            del self.db
        if hasattr(self, 'orientations'):
            del self.orientations
        self.get_logger().info("Resources successfully released.")

    def computeBasePlacementCallback(self, request, response):
        try:
            if not request.targets:
                response.success = False
                response.message = "[ERROR] No targets received in request."
                self.get_logger().error(response.message)
                return response

            # Convert ROS2 poses to appropriate format (pos, rot matrix)
            example_targets = []
            for pose in request.targets:
                pos = np.array([pose.position.x, pose.position.y, pose.position.z])
                quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
                rot = R.from_quat(quat).as_matrix()
                example_targets.append((pos, rot))

            # Generating global grid
            x_vals, y_vals, _, _ = self.define_global_grid(example_targets, cart_step=self.cart_step, global_size=self.global_size)

            # Creating obstacles (NEEDS TO be parameterized)
            occupancy_map = np.ones((len(y_vals), len(x_vals)), dtype=np.uint8)
            occupancy_map = self.add_obstacle_by_coords(occupancy_map, x_vals, y_vals, x_min=0.5, y_min=0.5, x_max=1.5, y_max=3.0)
            occupancy_map = self.add_obstacle_by_coords(occupancy_map, x_vals, y_vals, x_min=1.5, y_min=1.5, x_max=2.0, y_max=2.0)

            # Evaluating base positions on grid
            union_map, intersection_map = self.evaluate_base_positions_on_grid(
                example_targets, self.db, self.orientations,
                x_vals, y_vals,
                cart_step=self.cart_step,
                area_size=self.area_size,
                cart_min=self.cart_min,
                occupancy_map=occupancy_map
            )

            # Plotting results
            self.plot_score_map(union_map, x_vals, y_vals, title='Mapa de unión con obstáculos', ee_targets=example_targets, occupancy_map=occupancy_map)

            ## Obtaining best bases by score
            # top_union, max_score_union = self.get_top_bases(intersection_map, x_vals, y_vals, top_n=2)
            # self.get_logger().info(f"\n Best found bases (max score = {int(max_score_union)}):")
            # for i, (x, y) in enumerate(top_union[:2]):
            #     self.get_logger().info(f"B{i+1} → x: {x:.3f} m, y: {y:.3f} m")

            # Optimal base by centering and distance
            optimal_base, optimal_score = self.select_optimal_base(intersection_map, x_vals, y_vals,
                                                                example_targets,
                                                                min_distance=0.5,
                                                                perpendicular_tol=0.1)
            
            if optimal_base is None or np.any(np.isnan(optimal_base)):
                response.success = False
                response.message = "[ERROR] No valid base position found."
                self.get_logger().error(response.message)
                return response

            # Visualizing optimal base
            self.plot_score_map(intersection_map, x_vals, y_vals,
                        title='Mapa de intersección con obstaculos y con base óptima personalizada',
                        ee_targets=example_targets,
                        top_bases=[optimal_base] if optimal_base else None,
                        occupancy_map=occupancy_map)
    
            # Converting result to geometry_msgs/Pose
            pose = Pose()
            pose.position.x = float(optimal_base[0])  # Asignar las coordenadas de la base
            pose.position.y = float(optimal_base[1])
            pose.position.z = 0.0  # Puedes ajustar esto según tu necesidad
            pose.orientation = request.targets[0].orientation
            
            response.best_base = pose
            response.success = True
            response.message = (
                f"Computed optimal base with min distance and centering rules. "
                f"Best base calculated: (x: {pose.position.x:.2f} m, y: {pose.position.y:.2f} m)"
            )
            self.get_logger().info(response.message)

        except Exception as e:
            response.success = False
            response.message = f"Error during base placement computation: {str(e)}"
            self.get_logger().error(response.message)

        return response

    ## Auxiliar functions

    def load_database(self, db_path, orientations_path):
        with open(db_path, 'rb') as f:
            db = pickle.load(f)
            self.get_logger().info('[INFO] Database loaded.')
        all_voxels = list(db.keys())
        min_idx = np.min(all_voxels, axis=0)
        max_idx = np.max(all_voxels, axis=0)
        self.get_logger().info(f"[DEBUG] DB index range: X:{min_idx[0]}-{max_idx[0]}, Y:{min_idx[1]}-{max_idx[1]}, Z:{min_idx[2]}-{max_idx[2]}")
        with open(orientations_path, 'rb') as f:
            orientations = pickle.load(f)
            self.get_logger().info('[INFO] Orientations loaded.')
        return db, orientations
    
    def orientation_similarity(self, o1, o2):
        z1 = R.from_matrix(o1).apply([0, 0, 1])
        z2 = R.from_matrix(o2).apply([0, 0, 1])
        angle = np.arccos(np.clip(np.dot(z1, z2), -1.0, 1.0))
        return angle

    def find_similar_orientations(self, required_rot, orientation_list, threshold_rad=np.deg2rad(30)):
        similar_indices = []
        for i, R_matrix in enumerate(orientation_list):
            if ( self.orientation_similarity(required_rot, R_matrix) < threshold_rad ):
                similar_indices.append(i)
        return similar_indices

    def define_global_grid(self, ee_targets, cart_step=0.05, global_size=4.0):
        ee_positions = np.array([pos[:2] for pos, _ in ee_targets])
        center = np.mean(ee_positions, axis=0)
        half_size = global_size / 2
        global_min = center - half_size
        global_max = center + half_size
        x_vals = np.arange(global_min[0], global_max[0] + cart_step, cart_step)
        y_vals = np.arange(global_min[1], global_max[1] + cart_step, cart_step)
        return x_vals, y_vals, global_min, global_max

    def add_obstacle_by_coords(self, occupancy_map, x_vals, y_vals, x_min, y_min, x_max, y_max):
        j_min = np.searchsorted(x_vals, x_min, side='left')
        j_max = np.searchsorted(x_vals, x_max, side='right')
        i_min = np.searchsorted(y_vals, y_min, side='left')
        i_max = np.searchsorted(y_vals, y_max, side='right')

        i_min = max(0, i_min)
        i_max = min(len(y_vals), i_max)
        j_min = max(0, j_min)
        j_max = min(len(x_vals), j_max)

        occupancy_map[i_min:i_max, j_min:j_max] = 0
        return occupancy_map

    def evaluate_base_positions_on_grid(self, ee_targets, db, orientations, x_vals, y_vals, cart_step=0.05, area_size=1.6 * 2, cart_min=-1.6, occupancy_map=None):
        H, W = len(y_vals), len(x_vals)
        union_map = np.zeros((H, W))
        intersection_votes = np.zeros((H, W), dtype=int)
        half_cells = int(area_size / (2 * cart_step))

        for ee_pos, ee_rot in ee_targets:
            local_votes = np.zeros((H, W), dtype=int)
            cx = np.argmin(np.abs(x_vals - ee_pos[0]))
            cy = np.argmin(np.abs(y_vals - ee_pos[1]))

            for i in range(cy - half_cells, cy + half_cells):
                for j in range(cx - half_cells, cx + half_cells):
                    if not (0 <= i < H and 0 <= j < W):
                        continue
                    if occupancy_map is not None and occupancy_map[i, j] == 0:
                        continue

                    base_pos = np.array([x_vals[j], y_vals[i], 0.0])
                    rel_pos = ee_pos[:3] - base_pos
                    voxel_idx = tuple(np.round((rel_pos - (cart_min + cart_step / 2)) / cart_step).astype(int))
                    
                    # print(f"voxel_idx: {voxel_idx}, rel_pos: {rel_pos}, base_pos: {base_pos}")

                    if all(v >= 0 for v in voxel_idx) and voxel_idx in db:
                        similar_orients = self.find_similar_orientations(ee_rot, orientations)
                        # print(f"similar_orients indices count: {len(similar_orients)}")
                        score = sum(len(db[voxel_idx].get(idx, [])) for idx in similar_orients)
                        union_map[i, j] += score
                        if score > 0:
                            local_votes[i, j] = 1

            intersection_votes += local_votes

        intersection_map = np.where(intersection_votes == len(ee_targets), union_map, 0)

        if np.max(union_map) == 0:
            print("[ERROR] Union map is empty. Verify threshold of orientation or the databse.")
        if np.max(intersection_map) == 0:
            print("[ERROR] Intersection map is empty. Verify threshold of orientation or the databse.")
        return union_map, intersection_map
    
    def get_top_bases(self, score_map, x_vals, y_vals, top_n=5):
        flat = score_map.ravel()
        indices = np.argsort(flat)[::-1]
        coords = []
        max_score = flat[indices[0]]
        for idx in indices:
            if len(coords) >= top_n:
                break
            i, j = np.unravel_index(idx, score_map.shape)
            if score_map[i, j] == max_score:
                coords.append((x_vals[j], y_vals[i]))
        return coords, max_score

    def select_optimal_base(self, score_map, x_vals, y_vals, ee_targets, min_distance=0.3, perpendicular_tol=0.025):
        max_score = np.max(score_map)
        candidates = np.argwhere(score_map == max_score)

        targets_xy = np.array([pos[:2] for pos, _ in ee_targets])
        mean_targets = np.mean(targets_xy, axis=0)
        centered = targets_xy - mean_targets
        _, _, vh = np.linalg.svd(centered)
        principal = vh[0]
        perpendicular = np.array([-principal[1], principal[0]])

        best_score = np.inf
        best_coord = None

        for i, j in candidates:
            x = x_vals[j]
            y = y_vals[i]
            base_xy = np.array([x, y])

            dists = np.linalg.norm(targets_xy - base_xy, axis=1)
            if np.any(dists < min_distance):
                continue

            vec = base_xy - mean_targets
            offset_along_perp = np.dot(vec, perpendicular)
            offset_along_main = np.dot(vec, principal)

            if abs(offset_along_main) > perpendicular_tol:
                continue

            dist_avg = np.mean(dists)

            if dist_avg < best_score:
                best_score = dist_avg
                best_coord = (x, y)

        return best_coord, best_score

    def plot_score_map(self, score_map, x_vals, y_vals, title='Mapa', ee_targets=None, top_bases=None, occupancy_map=None):
        extent = [x_vals[0], x_vals[-1], y_vals[0], y_vals[-1]]
        plt.figure(figsize=(8, 6))
        plt.imshow(score_map, origin='lower', extent=extent, cmap='viridis')

        if occupancy_map is not None:
            masked = np.ma.masked_where(occupancy_map == 1, occupancy_map)
            plt.imshow(masked, origin='lower', extent=extent, cmap='Greys', alpha=0.4)

        if ee_targets:
            for i, (pos, _) in enumerate(ee_targets):
                plt.plot(pos[0], pos[1], 'rx')
                plt.text(pos[0] + 0.02, pos[1] + 0.02, f'P{i+1}', color='white')

        if top_bases:
            for i, (x, y) in enumerate(top_bases):
                plt.plot(x, y, 'go')
                plt.text(x + 0.02, y - 0.02, f'B{i+1}', color='lime')

        plt.colorbar(label='Número total de soluciones')
        plt.title(title)
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.grid(True)
        plt.tight_layout()
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = BasePlacementNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        node.get_logger().info("Shutting down node due to keyboard interrupt (Ctrl+C)")
        node.cleanup()
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()

if __name__ == '__main__':
    main()
