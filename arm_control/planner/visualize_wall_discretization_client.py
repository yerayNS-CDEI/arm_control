#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from robotic_arm_planner_interfaces.srv import ComputeWallDiscretization
from geometry_msgs.msg import Pose

import numpy as np
import matplotlib
matplotlib.use('TkAgg')  # o 'Qt5Agg' si usas Qt
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


class WallVisualizer(Node):
    def __init__(self):
        super().__init__('wall_discretization_client')
        self.cli = self.create_client(ComputeWallDiscretization, 'compute_wall_discretization')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')

        self.req = ComputeWallDiscretization.Request()
        self.req.start_point.x = -0.6
        self.req.start_point.y = 0.5
        self.req.start_point.z = 0.0
        self.req.end_point.x = 4.2
        self.req.end_point.y = 3.0
        self.req.end_point.z = 3.6
        self.req.robot_amplitude_range = 1.3
        self.req.robot_height_range = 1.3
        self.req.sensors_amplitude_range = 0.4
        self.req.sensors_height_range = 0.4
        self.req.resolution = 0.1
        self.req.target_i = 0
        self.req.target_j = 0

        self.future = self.cli.call_async(self.req)
        self.future.add_done_callback(self.handle_response)

    def handle_response(self, future):
        print("[DEBUG] Callback entered")
        try:
            res = future.result()
        except Exception as e:
            self.get_logger().error(f"Future failed: {e}")
            return

        if not res.success:
            self.get_logger().error('Discretization failed.')
            return

        wall_centers = np.array([[p.position.x, p.position.y, p.position.z] for p in res.wall_panels_centers])
        wall_vertices = np.array([[p.position.x, p.position.y, p.position.z] for p in res.wall_panels_vertices])
        cell_centers = np.array([[p.position.x, p.position.y, p.position.z] for p in res.panel_cells_centers])
        cell_vertices = np.array([[p.position.x, p.position.y, p.position.z] for p in res.panel_cells_vertices])

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.set_title('Wall and Subdivided Panel')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_box_aspect([1, 1, 1])
        ax.view_init(elev=25, azim=35)

        # Plot wall panels
        n_wall_quads = len(wall_vertices) // 4
        cmap = plt.get_cmap('viridis', n_wall_quads)
        for i in range(n_wall_quads):
            quad = wall_vertices[i * 4: (i + 1) * 4]
            poly = Poly3DCollection([quad], facecolors=cmap(i), alpha=0.5, edgecolors='k')
            ax.add_collection3d(poly)

        # Plot cell panels in blue
        n_cells = len(cell_vertices) // 4
        for i in range(n_cells):
            quad = cell_vertices[i * 4: (i + 1) * 4]
            poly = Poly3DCollection([quad], facecolors='cyan', alpha=0.5, edgecolors='b')
            ax.add_collection3d(poly)

        # Plot centers
        ax.scatter(wall_centers[:, 0], wall_centers[:, 1], wall_centers[:, 2], c='k', s=8, label="wall centers")
        ax.scatter(cell_centers[:, 0], cell_centers[:, 1], cell_centers[:, 2], c='blue', s=6, label="sensor centers")
        
        plt.legend()
        print("[DEBUG] ready to show plot")
        plt.show(block=True)

def main(args=None):
    rclpy.init(args=args)
    node = WallVisualizer()
    
    while rclpy.ok() and not node.future.done():
        rclpy.spin_once(node, timeout_sec=0.1)
    
    if node.future.done():
        try:
            result = node.future.result()
            node.handle_response(node.future)
        except Exception as e:
            node.get_logger().error(f"Service call failed: {e}")
    else:
        node.get_logger().error("Service call did not complete")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
