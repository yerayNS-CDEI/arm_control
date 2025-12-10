#
# Node to generate a service for the discretization of the walls to scan.
# It receives 2 points of opposite vertices of the wall (bottom left corner and top right corner)
#

import rclpy
from rclpy.node import Node
from arm_control.srv import ComputeWallDiscretization
from geometry_msgs.msg import Point, Pose

import numpy as np
import collections
import itertools

def prime_factors(n):
    i = 2
    while i * i <= n:
        if n % i == 0:
            n /= i
            yield i
        else:
            i += 1
    if n > 1:
        yield n

def prod(iterable):
    result = 1
    for i in iterable:
        result *= i
    return result

def get_divisors(n):
    pf = prime_factors(n)
    pf_with_multiplicity = collections.Counter(pf)
    powers = [[factor ** i for i in range(count + 1)] for factor, count in pf_with_multiplicity.items()]
    for prime_power_combo in itertools.product(*powers):
        yield prod(prime_power_combo)

def divide_plane(base_point, dir_ampl, dir_height, total_length, total_height,
                 range_ampl, range_height, resolution_h, resolution_v, dec_round):
    tolerance = 10 ** (-dec_round)
    int_length = int(np.round(total_length/resolution_h, 0))
    int_height = int(np.round(total_height/resolution_v, 0))

    h_div = np.array(sorted(list(get_divisors(int_length)))) * resolution_h
    h_valid = h_div[(h_div <= range_ampl) & (h_div > 0)]
    if h_valid.size == 0:
        raise ValueError("No horizontal valid divisions.")
    best_step_h = np.max(h_valid)
    n_cols = round(total_length / best_step_h)

    v_div = np.array(sorted(list(get_divisors(int_height)))) * resolution_v
    v_valid = v_div[(v_div <= range_height) & (v_div > 0)]
    if v_valid.size == 0:
        raise ValueError("No vertical valid divisions.")
    best_step_v = np.max(v_valid)
    n_rows = round(total_height / best_step_v)

    cells = [[None for _ in range(n_cols)] for _ in range(n_rows)]
    centers = [[None for _ in range(n_cols)] for _ in range(n_rows)]

    for i in range(n_rows):
        for j in range(n_cols):
            corner = (np.array(base_point) +
                      j * best_step_h * np.array(dir_ampl) +
                      i * best_step_v * np.array(dir_height))

            p1 = corner
            p2 = p1 + best_step_h * np.array(dir_ampl)
            p3 = p2 + best_step_v * np.array(dir_height)
            p4 = p1 + best_step_v * np.array(dir_height)

            quad = np.array([p1, p2, p3, p4])
            cells[i][j] = quad
            centers[i][j] = np.mean(quad, axis=0)

    return cells, centers

def to_pose(position):
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = position.tolist()
    pose.orientation.w = 1.0  # no rotation
    return pose

class WallDiscretizer(Node):
    def __init__(self):
        super().__init__('wall_discretizer_node')
        self.srv = self.create_service(ComputeWallDiscretization, 'compute_wall_discretization', self.discretize_wall_callback)
        self.get_logger().info("Wall discretization node initialized. Service is already available.")

    def discretize_wall_callback(self, request, response):
        try:
            p1 = np.array([request.start_point.x, request.start_point.y, request.start_point.z])
            p2 = np.array([request.end_point.x, request.end_point.y, request.end_point.z])

            dir_xy = p2[:2] - p1[:2]
            wall_length = np.linalg.norm(dir_xy)
            unit_xy = dir_xy / wall_length
            dir_vector = np.array([*unit_xy, 0])
            wall_height = abs(p2[2] - p1[2])
            height_vector = np.array([0, 0, 1])

            dec_round = 1
            resolution = request.resolution
            h_res = v_res = resolution

            wall_cells, wall_centers = divide_plane(
                p1, dir_vector, height_vector,
                wall_length, wall_height,
                request.robot_amplitude_range,
                request.robot_height_range,
                h_res, v_res, dec_round)

            # Wall panel centers
            for row in wall_centers:
                for c in row:
                    response.wall_panels_centers.append(to_pose(c))

            # Wall panel vertices (flattened)
            for row in wall_cells:
                for cell in row:
                    for v in cell:
                        response.wall_panels_vertices.append(to_pose(v))

            # Selecting indicated panel
            try:
                target_panel = wall_cells[request.target_i][request.target_j]
            except IndexError:
                self.get_logger().error("Invalid panel indices")
                response.success = False
                return response
            
            # Divide panel in subcells
            v_ampl = target_panel[1] - target_panel[0]
            v_height = target_panel[3] - target_panel[0]
            unit_ampl = v_ampl / np.linalg.norm(v_ampl)
            unit_height = v_height / np.linalg.norm(v_height)
            
            sensor_cells, sensor_centers = divide_plane(
                target_panel[0], unit_ampl, unit_height,
                np.linalg.norm(v_ampl), np.linalg.norm(v_height),
                request.sensors_amplitude_range, request.sensors_height_range,
                h_res, v_res, dec_round)

            for row in sensor_centers:
                for c in row:
                    response.panel_cells_centers.append(to_pose(c))

            for row in sensor_cells:
                for cell in row:
                    for v in cell:
                        response.panel_cells_vertices.append(to_pose(v))

            response.success = True
            self.get_logger().info(f"Wall of size {wall_length:.2f} x {wall_height:.2f} m divided into {len(wall_centers[0])} cols and {len(wall_centers)} rows")
            self.get_logger().info(f"Panel ({request.target_i},{request.target_j}) divided into {len(sensor_centers[0])} cols and {len(sensor_centers)} rows")

            return response

        except Exception as e:
            response.success = False
            self.get_logger().error(f"Error: {e}")
            return response

def main(args=None):
    rclpy.init(args=args)
    node = WallDiscretizer()
    rclpy.spin(node)
    rclpy.shutdown()
