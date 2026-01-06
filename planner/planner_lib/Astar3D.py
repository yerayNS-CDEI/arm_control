from typing import List, Tuple, Dict, Set
import numpy as np
import heapq
from math import sqrt

import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D  # Not needed - 3D projection auto-registers in matplotlib 3.1+

from scipy.ndimage import binary_dilation

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

def custom_format(array):
    formatted = "[" + "; ".join(f"[{a}, {b}, {c}]" for a, b, c in array) + "]"
    print(formatted)

def create_node(position: Tuple[int, int, int], g: float = float('inf'), 
                h: float = 0.0, parent: Dict = None) -> Dict:
    """
    Create a node for the A* algorithm.
    
    Args:
        position: (x, y) coordinates of the node
        g: Cost from start to this node (default: infinity)
        h: Estimated cost from this node to goal (default: 0)
        parent: Parent node (default: None)
    
    Returns:
        Dictionary containing node information
    """
    return {
        'position': position,
        'g': g,
        'h': h,
        'f': g + h,
        'parent': parent
    }

def calculate_heuristic(pos1: Tuple[int, int, int], pos2: Tuple[int, int, int]) -> float:
    """
    Calculate the estimated distance between two points using Euclidean distance.
    If other heuristic is used (like Manhattan distance) it's possible that the diagonal
    moves are most likely chosen, this can be avoided by adding a tiny penalty to them. 
    """
    x1, y1, z1 = pos1
    x2, y2, z2 = pos2
    return sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)

def get_valid_neighbors(grid: np.ndarray, position: Tuple[int, int, int]) -> List[Tuple[int, int, int]]:
    """
    Get all valid neighboring positions in the grid.
    
    Args:
        grid: 2D numpy array where 0 represents walkable cells and 1 represents obstacles
        position: Current position (x, y)
    
    Returns:
        List of valid neighboring positions
    """
    x, y, z = position
    rows, cols, depth = grid.shape
    
    # All possible moves (including diagonals)
    possible_moves = [
        (x+1, y, z), (x-1, y, z),    # Right, Left
        (x, y+1, z), (x, y-1, z),    # Forward, Backward
        (x+1, y+1, z), (x-1, y-1, z),  # Diagonal moves x-y
        (x+1, y-1, z), (x-1, y+1, z),
        (x, y, z+1), (x, y, z-1),   # Up, Down
        (x+1, y, z+1), (x-1, y, z+1),   # Diagonal moves x-z
        (x+1, y, z-1), (x-1, y, z-1),
        (x, y+1, z+1), (x, y-1, z+1),   # Diagonal moves y-z
        (x, y+1, z-1), (x, y-1, z-1),
        (x+1, y+1, z+1), (x+1, y-1, z+1),   # Corner moves
        (x-1, y+1, z+1), (x-1, y-1, z+1),
        (x+1, y+1, z-1), (x+1, y-1, z-1),
        (x-1, y+1, z-1), (x-1, y-1, z-1),
    ]   
    
    return [    # this rows/cols could be either int or real coordinates ??
        (nx, ny, nz) for nx, ny, nz in possible_moves
        if 0 <= nx < rows and 0 <= ny < cols and 0 <= nz < depth  # Within grid bounds
        and grid[nx, ny, nz] == 0                # Not an obstacle
    ]

def reconstruct_path(goal_node: Dict) -> List[Tuple[int, int, int]]:
    """
    Reconstruct the path from goal to start by following parent pointers.
    """
    path = []
    current = goal_node
    
    while current is not None:
        path.append(current['position'])
        current = current['parent']
        
    return path[::-1]  # Reverse to get path from start to goal

def find_path(grid: np.ndarray, start: Tuple[int, int, int], 
              goal: Tuple[int, int, int]) -> List[Tuple[int, int, int]]:
    """
    Find the optimal path using A* algorithm.
    
    Args:
        grid: 2D numpy array (0 = free space, 1 = obstacle)
        start: Starting position (x, y)
        goal: Goal position (x, y)
    
    Returns:
        List of positions representing the optimal path
    """
    # Initialize start node
    start_node = create_node(
        position=start,
        g=0,
        h=calculate_heuristic(start, goal)
    )
    
    # Initialize open and closed sets
    open_list = [(start_node['f'], start)]  # Priority queue
    open_dict = {start: start_node}         # For quick node lookup
    closed_set = set()                      # Explored nodes
    
    while open_list:
        # Get node with lowest f value
        _, current_pos = heapq.heappop(open_list)
        current_node = open_dict[current_pos]
        
        # Check if we've reached the goal
        if current_pos == goal:
            return reconstruct_path(current_node)
            
        closed_set.add(current_pos)
        
        # Explore neighbors
        for neighbor_pos in get_valid_neighbors(grid, current_pos):
            # Skip if already explored
            if neighbor_pos in closed_set:
                continue
                
            # Calculate new path cost
            tentative_g = current_node['g'] + calculate_heuristic(current_pos, neighbor_pos)
            
            # Create or update neighbor
            if neighbor_pos not in open_dict:
                neighbor = create_node(
                    position=neighbor_pos,
                    g=tentative_g,
                    h=calculate_heuristic(neighbor_pos, goal),
                    parent=current_node
                )
                heapq.heappush(open_list, (neighbor['f'], neighbor_pos))
                open_dict[neighbor_pos] = neighbor
            elif tentative_g < open_dict[neighbor_pos]['g']:
                # Found a better path to the neighbor
                neighbor = open_dict[neighbor_pos]
                neighbor['g'] = tentative_g
                neighbor['f'] = tentative_g + neighbor['h']
                neighbor['parent'] = current_node
    
    return []  # No path found

def visualize_path(grid: np.ndarray, path: List[Tuple[int, int, int]]):
    """
    Visualize the grid and found path.
    """
    fig = plt.figure()
    ax = plt.axes(projection='3d')

    # Get coordinates of obstacle cells (value == 1)
    obstacle_y, obstacle_x, obstacle_z = np.where(grid == 1)
    ax.scatter(obstacle_y, obstacle_x, obstacle_z, c='black', marker='s',s=50,label='Obstacles')
    
    # Get coordinates of path
    if path:
        path = np.array(path)
        x = path[:, 0]
        y = path[:, 1]
        z = path[:, 2]
        ax.plot(x, y, z, c='r', marker='o', label='Path')
    
    # plt.grid(True)
    ax.legend(fontsize=12)
    ax.set_title("A* Pathfinding Result")
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    # plt.show(block=False)

    return fig, ax, path

###################
### Use Example ###
###################

# # Create a sample grid
# grid = np.zeros((20, 20, 20))  # 20x20 grid, all free space initially
# # Add some obstacles
# grid[5:15, 10, 5:15] = 1  # Vertical wall
# # grid[5:15, 10, 10:18] = 1  # Vertical wall
# grid[5, 5:15, 5:15] = 1   # Horizontal wall
# # Define start and goal positions
# start_pos = (2, 7, 12)  # x and y coordinates are inverted !!
# goal_pos = (18, 18, 8)
# # Find the path
# path = find_path(grid, start_pos, goal_pos)
# print(path[:])

# if path:
#     print(f"Path found with {len(path)} steps!")
#     # custom_format(path)
#     visualize_path(grid, path)
# else:
#     print("No path found!")