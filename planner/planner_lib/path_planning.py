import os
import sys
import time
import numpy as np
from exp_utils import robot_types
from rm4d.robots import Simulator
import matplotlib.pyplot as plt
import matplotlib.cm as cm

from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
from scipy.ndimage import binary_dilation

from closed_form_algorithm import closed_form_algorithm

import pybullet as p

from numpy.linalg import norm

# Add the root folder (ws_reachability) to sys.path
current_dir = os.path.dirname(os.path.abspath(__file__))
root_dir = os.path.abspath(os.path.join(current_dir, "../../"))
sys.path.append(root_dir)

from Astar3D import find_path, visualize_path, custom_format

def dilate_obstacles(occupancy_grid, dilation_distance, x_vals):
    # Create a structuring element for dilation (3D cube of size dilation_distance)
    dilation_size = int(np.ceil(dilation_distance / (x_vals[1] - x_vals[0])))  # Convert distance to grid units
    struct_element = np.ones((dilation_size, dilation_size, dilation_size), dtype=np.uint8)
    
    # Perform 3D dilation
    dilated_grid = binary_dilation(occupancy_grid, structure=struct_element).astype(np.uint8)
    return dilated_grid

def world_to_grid(x, y, z, x_vals, y_vals, z_vals):
    """
    Converts real-world coordinates (like x = 0.35, y = -0.62) into 
    their corresponding grid indices (i, j) in the 2D array.
    x,y: real-world coordinates
    x_vals,y_vals: 1D arrays of grid coordinates along x and y (create_2d_grid)
    """
    i = np.argmin(np.abs(x_vals - x))   # find closest x in the grid
    j = np.argmin(np.abs(y_vals - y))   # find closest y in the grid
    k = np.argmin(np.abs(z_vals - z))   # find closest z in the grid
    return i, j, k

# i, j = world_to_grid(0.0, -0.5, x_vals, y_vals)
# print(i, j)  # might print something like (381, 208)
# print(grid[381][208])

def grid_to_world(i, j, k, x_vals, y_vals, z_vals):
    """
    Converts grid indices (i, j) into their corresponding real-world coordinates (x, y).
    i, j: grid indices
    x_vals, y_vals: 1D arrays of grid coordinates along x and y (create_2d_grid)
    """
    x = x_vals[i]  # Get the x-coordinate from the grid
    y = y_vals[j]  # Get the y-coordinate from the grid
    z = z_vals[k]  # Get the y-coordinate from the grid
    return x, y, z

def align_path_to_world(path_local, base_pos=[0.0, 0.0, 0.01], flip_xy=True, R_custom=None):
    """
    Transforms a path from local robot base frame to PyBullet world frame.

    Parameters:
    - path_local: List of (x, y, z) tuples
    - base_pos: Translation offset to align to world frame
    - flip_xy: If True, flips X and Y axes (applies diag([-1, -1, 1]))
    - R_custom: Optional custom rotation matrix (3x3)

    Returns:
    - path_world: List of transformed (x, y, z) points
    """
    t = np.array(base_pos)
    if R_custom is not None:
        R = np.array(R_custom)
    elif flip_xy:
        R = np.diag([-1, -1, 1])  # flip X and Y
    else:
        R = np.eye(3)

    path_world = [(R @ np.array(p) + t).tolist() for p in path_local]
    return path_world

def scale_path(path, scale_factor):
    return [(x * scale_factor, y * scale_factor, z * scale_factor) for (x, y, z) in path]

# Ensure that the loaded file is a dictionary
robot_name = 'ur10e'
filename = "reachability_map_27_fused"
fn_npy = f"{filename}.npy"
reachability_map_fn = os.path.join('data',f'eval_poses_{robot_name}',fn_npy)
reachability_map = np.load(reachability_map_fn, allow_pickle=True).item()

# Extract grid size and resolution
parts = filename.split('_')
grid_size = int(parts[2])

# Simulate the robot setup
sim = Simulator(with_gui=False)
robot = robot_types[robot_name](sim)
radius = robot.range_radius
x_min = -radius
x_max = radius
y_min = -radius
y_max = radius

# Calculate resolution and x, y values
resolution = (x_max - x_min) / grid_size
x_vals = np.linspace(x_min + (resolution / 2), x_max - (resolution / 2), grid_size)
y_vals = np.linspace(y_min + (resolution / 2), y_max - (resolution / 2), grid_size)

reach_data = []
for z_value, reachability_slice in reachability_map.items():
    # Loop over the reachability slice and store (x, y, z_value, reachability)
    for i, x in enumerate(x_vals):
        for j, y in enumerate(y_vals):
            reachability_value = reachability_slice[i, j]  # The reachability value at (x, y)
            reach_data.append([x, y, z_value, reachability_value])

reach_data = np.array(reach_data)  # shape: (N, 4) where columns are x, y, z, reachability

nonzero_mask = reach_data[:, 3] != 0
filtered_map = reach_data[nonzero_mask]

reach_x = filtered_map[:, 0]
reach_y = filtered_map[:, 1]
reach_z = filtered_map[:, 2]
reachability = filtered_map[:, 3]

# # Plotting the reachability map
# fig = plt.figure()
# ax = plt.axes(projection='3d')
# scatter = ax.scatter(reach_y, reach_x, reach_z, c=reachability, cmap='viridis', marker='o', s=50)
# # plt.figure()
# # plt.imshow(reachability_slice, cmap='hot', interpolation='nearest')
# # plt.colorbar(label='Reachability')
# ax.set_title("Reachability Map")
# ax.set_xlabel('X Label')
# ax.set_ylabel('Y Label')
# ax.set_zlabel('Z Label')
# fig.colorbar(scatter, ax=ax, label='Reachability')
# plt.show(block=False)

# # Wait for user interaction before closing all plots
# input("Press Enter to close all plots...")

###########################################################################
# Defining Obstacles & Defining grid (fusion reachability map + obstacles)
###########################################################################

# Get Z levels and determine shape of the occupancy map
z_levels = sorted(reachability_map.keys())
grid_shape = (grid_size, grid_size, len(z_levels))
occupancy_grid = np.zeros(grid_shape, dtype=np.uint8)

# Get the z-values similar to x_vals and y_vals
z_min = min(z_levels)
z_max = max(z_levels)
z_vals = np.linspace(z_min, z_max, len(z_levels))  # assumes uniform spacing

# 3D meshgrid of coordinates
X, Y, Z = np.meshgrid(x_vals, y_vals, z_vals, indexing='ij')  # shape: (grid_size, grid_size, num_z)

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

# Plotting the reachability map
fig = plt.figure()
ax = plt.axes(projection='3d')

# Reachability points (non-zero)
scatter = ax.scatter(reach_x, reach_y, reach_z, c=reachability, cmap='viridis', marker='o', s=10, alpha=0.1, label='Reachability')

# Obstacle points
# Get voxel coordinates where occupancy = 1
obstacle_indices = np.argwhere(occupancy_grid == 1)
obs_x = x_vals[obstacle_indices[:, 0]]
obs_y = y_vals[obstacle_indices[:, 1]]
obs_z = z_vals[obstacle_indices[:, 2]]

# Plot obstacles in red
ax.scatter(obs_x, obs_y, obs_z, c='red', marker='s', s=20, alpha=0.9, label='Obstacle')

# Labels and aesthetics
ax.set_title("Reachability Map with Obstacles")
ax.set_xlabel('Y')
ax.set_ylabel('X')
ax.set_zlabel('Z')
fig.colorbar(scatter, ax=ax, label='Reachability')
ax.legend()
plt.show(block=False)

# # Wait for user interaction before closing all plots
# input("Press Enter to close all plots...")

# Define the dilation distance (in meters, for example)
dilation_distance = 0.4  # Enlarge obstacles by 0.1 meters in all directions

# Apply dilation
occupancy_grid_dilated = dilate_obstacles(occupancy_grid, dilation_distance, grid_size)

# # Plotting the reachability map
# fig = plt.figure()
# ax = plt.axes(projection='3d')

# # Reachability points (non-zero)
# scatter = ax.scatter(reach_y, reach_x, reach_z, c=reachability, cmap='viridis', marker='o', s=10, alpha=0.2, label='Reachability')

# # Obstacle points
# # Get voxel coordinates where occupancy = 1 in the dilated occupancy grid
# obstacle_indices = np.argwhere(occupancy_grid_dilated == 1)
# obs_x = x_vals[obstacle_indices[:, 0]]
# obs_y = y_vals[obstacle_indices[:, 1]]
# obs_z = z_vals[obstacle_indices[:, 2]]

# # Plot obstacles in red
# ax.scatter(obs_y, obs_x, obs_z, c='red', marker='s', s=20, alpha=0.9, label='Enlarged Obstacle')

# # Labels and aesthetics
# ax.set_title("Reachability Map with Enlarged Obstacles")
# ax.set_xlabel('Y')
# ax.set_ylabel('X')
# ax.set_zlabel('Z')
# fig.colorbar(scatter, ax=ax, label='Reachability')
# ax.legend()
# plt.show(block=False)

# # Wait for user interaction before closing all plots
# input("Press Enter to close all plots...")

###########################################################################
# Defining start and goal positions (and orientations)
#           # We need to define an "ideal" position as a home for starting the closed form algorithms
###########################################################################

start_pos_world = [-0.25, 0.6, 0.2]  # [x,y,z]
goal_pos_world = [-0.25, -0.5, 0.6]   # [x,y,z]
start_position = world_to_grid(start_pos_world[0], start_pos_world[1], start_pos_world[2], x_vals, y_vals, z_vals)  # [i,j,k]
goal_position = world_to_grid(goal_pos_world[0], goal_pos_world[1], goal_pos_world[2], x_vals, y_vals, z_vals)      # [i,j,k]

###########################################################################
# Computing EE path (A* Algorithm)
###########################################################################
path = find_path(occupancy_grid_dilated, start_position, goal_position)     # in grid coordinates!!

path_world = []     # in world coordinates!!
x_coord = []
y_coord = []
z_coord = []
for i in range(len(path)):
    world_coord = grid_to_world(path[i][0],path[i][1],path[i][2],x_vals,y_vals,z_vals)
    ax.scatter(world_coord[0], world_coord[1], world_coord[2], c='black', marker='s', s=20, alpha=1.0)
    path_world.append(world_coord)
    x_coord.append(world_coord[0])
    y_coord.append(world_coord[1])
    z_coord.append(world_coord[2])
print('path_world: ',path_world)
# print('x_coord: ',x_coord)
plt.show(block=False)

if path:
    print(f"Path found with {len(path)} steps!")
    custom_format(path)
    visualize_path(occupancy_grid, path)
else:
    print("No path found!")

# # Wait for user interaction before closing all plots
# input("Press Enter to close all plots...")

###########################################################################
# Computing interpolated orientations
###########################################################################
start_orientation = []  # rotation matrix
goal_orientation = []   # rotation matrix

# Define the start and goal rotation matrices using Euler angles
start_rotation_matrix = R.from_euler('xyz', [60, 120, 150], degrees=True).as_matrix()
goal_rotation_matrix = R.from_euler('xyz', [0, -90, 0], degrees=True).as_matrix()
start_rotation = R.from_matrix(start_rotation_matrix)
goal_rotation = R.from_matrix(goal_rotation_matrix)
# print('start_rotation_matrix',start_rotation_matrix)
# print('start_rotation',start_rotation)
# print('goal_rotation_matrix',goal_rotation_matrix)

# Create a Rotation object that holds both start and goal rotations
key_rots = R.from_quat([start_rotation.as_quat(), goal_rotation.as_quat()])

# Define the times corresponding to the key rotations
key_times = [0, 1]  # t=0 for start, t=1 for goal

# Create the interpolator
slerp = Slerp(key_times, key_rots)

# Create interpolation times
num_interpolations = len(path)
interp_times = np.linspace(0, 1, num_interpolations)

# Compute interpolated rotations
interp_rots = slerp(interp_times)

# Convert to Euler angles
interpolated_euler_angles = interp_rots.as_euler('xyz', degrees=True)
print("Interpolated Euler angles:\n", interpolated_euler_angles)

# Optionally, convert to rotation matrices too
interpolated_rotation_matrices = interp_rots.as_matrix()
print('interpolated_rotation_matrices',interpolated_rotation_matrices)

# === Apply the rotations to a unit vector (e.g., z-axis) ===
origin_vector = np.array([0, 0, 1])
rotated_vectors = np.array([r.apply(origin_vector) for r in interp_rots])

# === Generate colors along a colormap (e.g., from red to blue) ===
colors = cm.jet(np.linspace(0, 1, num_interpolations))  # You can choose 'viridis', 'plasma', etc.

# === Plot the rotated vectors in 3D ===
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot each rotated vector with its color
for vec, color in zip(rotated_vectors, colors):
    ax.quiver(0, 0, 0, vec[0], vec[1], vec[2], color=color, length=0.8, normalize=True)

# Optionally: scatter endpoints of the vectors
ax.scatter(rotated_vectors[:, 0], rotated_vectors[:, 1], rotated_vectors[:, 2], c=colors, s=50)

ax.set_xlim([-1, 1])
ax.set_ylim([-1, 1])
ax.set_zlim([-1, 1])
ax.set_title("Interpolated Orientation Vectors (Start to Goal)")
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")

plt.show(block=False)
input("Press Enter to close all plots...")

###########################################################################
# Computing robot's joint values (Closed form algorithms)
###########################################################################

# EE coordinates
# x_coord = np.linspace(0.3, -0.3, num_interpolations)
x_coord = path_world[0][:]
y_coord = path_world[1][:]
z_coord = path_world[2][:]
# print('x_coord v2: ',x_coord)
all_joint_values = []

# Starting point
# start_position = np.array([x_coord[0], 0.5, 0.3])
start_position = np.array([path_world[0][0], path_world[0][1], path_world[0][2]])
start_rotation = interpolated_rotation_matrices[0]
start_orientation = np.eye(4)
start_orientation[:3, :3] = start_rotation
start_orientation[:3, 3] = start_position
home_position = np.array([0.0, -1.2, -2.3, -1.2, 1.57, 0.0])

# Initial robot state
q_current = closed_form_algorithm(start_orientation, np.array(home_position), type=0)
all_joint_values.append(q_current)
print(f"Step 0: q = {np.round(q_current, 4)}")

# Path computation
for i in range(1, interpolated_rotation_matrices.shape[0]):
    # print(f"Step {i}, Current joint values: {q_current}")  # Reemplaza esto con una visualización si quieres
    orientation = np.eye(4)
    orientation[:3, :3] = interpolated_rotation_matrices[i]
    # orientation[:3, 3] = np.array([x_coord[i], 0.5, 0.3])
    orientation[:3, 3] = np.array([path_world[i][0], path_world[i][1], path_world[i][2]])
    
    q_new = closed_form_algorithm(orientation, q_current, type=0)
    # print(orientation)
    all_joint_values.append(q_new)
    q_current = q_new

    print(f"Step {i}: q = {np.round(q_current, 4)}")
    # time.sleep(1)

print("Process finished")
print(len(path), len(rotated_vectors))

if path:
    fig, ax, path_array = visualize_path(occupancy_grid, path)
    path = np.array(path)    
    x = path[:, 0]
    y = path[:, 1]
    z = path[:, 2]
    for i, (vec, color) in enumerate(zip(rotated_vectors, colors)):
        ax.quiver(
            x[i], y[i], z[i],      # starting point
            vec[0], vec[1], vec[2],  # direction vector
            color=color,
            length=1,  # adjust for better scaling
            normalize=True
        )
    plt.show(block=False)
else:
    print("No path found!")

input("Joints computed...")

###########################################################################
# Presenting the results in the simulated environement
###########################################################################
# robot.joint_pos # current joint position
# robot.reset_joint_pos(q) # joint position to a given configuration

# Simulate the robot setup
robot_name = 'ur10e'
sim = Simulator(with_gui=True)
robot = robot_types[robot_name](sim)
input("Press Enter to continue...")
print(f"Initial/Home Position")
robot.reset_joint_pos(home_position)
time.sleep(2)

aabb_min, aabb_max = sim.bullet_client.getAABB(robot.robot_id)
size = [b - a for a, b in zip(aabb_min, aabb_max)]
print("Robot bounding box size (X,Y,Z):", size)

# shared_sphere_id = sim.bullet_client.createVisualShape(
#     p.GEOM_SPHERE, radius=0.01, rgbaColor=[0, 0.5, 1, 0.3])

# for i in range(len(reach_x)):
#     sim.add_sphere_v2([reach_x[i], reach_y[i], reach_z[i]], radius=0.01,
#                    color=[0, 0.5, 1], visual_shape_id=shared_sphere_id)
# input("Press Enter to continue...")

print("First path point:", path_world[0])
print("Robot base position:", robot.sim.bullet_client.getBasePositionAndOrientation(robot.robot_id)[0])

# Align the path before drawing
scale_factor = 1.48  # empirically determined
scaled_path = scale_path(path_world, scale_factor)
aligned_path = align_path_to_world(scaled_path)  # if you still need axis flip, etc.
print('aligned_path: ',aligned_path)

# Draw points
for pos in aligned_path:
    sim.add_sphere(pos=pos, radius=0.015, color=[0.1, 0.8, 0.2, 1.0])  # greenish

# Draw connecting lines
for i in range(len(aligned_path) - 1):
    sim.bullet_client.addUserDebugLine(aligned_path[i], aligned_path[i + 1],
                                       lineColorRGB=[0, 0, 1], lineWidth=2)
    
for i,q_current in enumerate(all_joint_values):
    robot.reset_joint_pos(q_current)
    print(f"Step {i}: q = {np.round(q_current, 4)}")
    # EE current position in world
    ee_pos, _ = sim.bullet_client.getLinkState(robot.robot_id, robot.end_effector_link_id)[:2]
    print('Step ',i,'. ee_pos:  ',ee_pos)
    # # First path point in world
    # sim.add_sphere(pos=aligned_path[0], radius=0.02, color=[1, 0, 0, 1])  # red
    # sim.add_sphere(pos=ee_pos, radius=0.02, color=[0, 0, 1, 1])  # blue
    time.sleep(1)
input("Press Enter to continue...")


input("Press Enter to finish program...")

# Creo que se deberian añadir tambien medidas de seguridad para que el 
# robot se detenga en caso de que aparezcan obstaculos en el camino.
# El entorno debe ser dinamico para detectar estos cambios.
# Una buena manera seria recalcular los puntos en que ahora el robot colisone
# o modificar el path cada vez que aparezcan nuevos obstaculos en el 
# reachability map calculado.
# Collision checking

# Tareas posteriores: añadirlo en nodo de ros para que se mueva el robot simulado
