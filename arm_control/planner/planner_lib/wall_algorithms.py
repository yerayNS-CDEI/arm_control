#
# Python code to generate the discretization of the walls to scan
#

import numpy as np
import matplotlib.pyplot as plt
import matplotlib
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

import collections
import itertools
import time

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

    powers = [
        [factor ** i for i in range(count + 1)]
        for factor, count in pf_with_multiplicity.items()
    ]

    for prime_power_combo in itertools.product(*powers):
        yield prod(prime_power_combo)

def divide_plane(base_point, dir_ampl, dir_height,
                 total_length, total_height,
                 range_ampl, range_height,
                 resolution_h, resolution_v,
                 dec_round):
    
    tolerance = 10 ** (-dec_round)

    # Redondeo de dimensiones
    int_length = int(np.round(total_length/resolution_h, 0))
    int_height = int(np.round(total_height/resolution_v, 0))

    # Candidatos horizontales
    h_div = np.array(sorted(list(get_divisors(int_length))))*resolution_h
    h_valid = h_div[(h_div <= range_ampl) & (h_div > 0)]
    print(f"[DEBUG] total_length = {total_length}")
    print(f"[DEBUG] h_steps = {h_div}")
    print(f"[DEBUG] h_valid = {h_valid}")

    if h_valid.size == 0:
        raise ValueError("No horizontal valid divisions.")
    best_step_h = np.max(h_valid)
    n_cols = round(total_length / best_step_h)

    # Candidatos verticales
    v_div = np.array(sorted(list(get_divisors(int_height))))*resolution_v
    v_valid = v_div[(v_div <= range_height) & (v_div > 0)]
    print(f"[DEBUG] total_height = {total_height}")
    print(f"[DEBUG] v_div = {v_div}")
    print(f"[DEBUG] v_valid = {v_valid}")

    if v_valid.size == 0:
        raise ValueError("No vertical valid divisions.")
    best_step_v = np.max(v_valid)
    n_rows = round(total_height / best_step_v)

    # Inicialización de celdas y centros
    cells = [[None for _ in range(n_cols)] for _ in range(n_rows)]
    centers = [[None for _ in range(n_cols)] for _ in range(n_rows)]

    # Construcción de las celdas
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

    return cells, centers, n_rows, n_cols, best_step_h, best_step_v, h_valid, v_valid

# Parámetros generales
dec_round = 1
h_resolution = 1/(10**dec_round)
v_resolution = 1/(10**dec_round)

# Características del robot y sensores
robot_amplitude_range = 1.3
robot_height_range = 1.3
sensors_amplitude_range = 0.4
sensors_height_range = 0.4

# Puntos del muro
point1 = np.array([-0.6, 0.5, 0])
point2 = np.array([4.2, 3.0, 3.6])

# Cálculo de vectores base
dir_xy = point2[:2] - point1[:2]
wall_length = np.linalg.norm(dir_xy)
unit_xy = dir_xy / wall_length
dir_vector = np.array([*unit_xy, 0])
wall_height = abs(point2[2] - point1[2])
height_vector = np.array([0, 0, 1])
print(f"Wall with height {wall_height:.2f} m and longitude {wall_length:.2f} m.")

# División del muro en paneles
wall_cells, wall_centers, n_rows, n_cols, step_h, step_v, _, _ = divide_plane(
    point1, dir_vector, height_vector,
    wall_length, wall_height,
    robot_amplitude_range, robot_height_range,
    h_resolution, v_resolution, dec_round)

print(f"Wall divided into {n_cols} columns of {step_h:.2f} m and {n_rows} rows of {step_v:.2f} m.")

# Visualización del muro
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_title('Wall divided into panels')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_box_aspect([1, 1, 1])  # Relación de aspecto igualada
ax.view_init(elev=25, azim=35)  # Vista 3D inclinada

cmap = plt.get_cmap('viridis', n_rows * n_cols)
idx = 0

for i in range(n_rows):
    for j in range(n_cols):
        verts = [wall_cells[i][j]]  # must be list of arrays
        poly = Poly3DCollection(verts, facecolors=cmap(idx), alpha=0.6, edgecolors='k')
        ax.add_collection3d(poly)
        center = wall_centers[i][j]
        ax.scatter(*center, c='k', s=10)
        idx += 1

# Selección del panel
target_i = 0
target_j = 0
area = wall_cells[target_i][target_j]
v_ampl = area[1] - area[0]
v_height = area[3] - area[0]
unit_ampl = v_ampl / np.linalg.norm(v_ampl)
unit_height = v_height / np.linalg.norm(v_height)

# División del panel en celdas sensoras
sensor_cells, sensor_centers, n_v, n_h, step_h, step_v, _, _ = divide_plane(
    area[0], unit_ampl, unit_height,
    np.linalg.norm(v_ampl), np.linalg.norm(v_height),
    sensors_amplitude_range, sensors_height_range,
    h_resolution, v_resolution, dec_round)

print(f"Panel ({target_i+1},{target_j+1}) divided into {n_h} columns of {step_h:.2f} m and {n_v} rows of {step_v:.2f} m.")

# Visualización de celdas sensoras
fig2 = plt.figure()
ax2 = fig2.add_subplot(111, projection='3d')
ax2.set_title(f'Panel subdivision ({target_i+1},{target_j+1})')
ax2.set_xlabel('X')
ax2.set_ylabel('Y')
ax2.set_zlabel('Z')
ax2.set_box_aspect([1, 1, 1])  # Relación de aspecto igualada
ax2.view_init(elev=25, azim=35)  # Vista 3D inclinada

for i in range(n_v):
    for j in range(n_h):
        s = sensor_cells[i][j]
        center = sensor_centers[i][j]

        verts = np.asarray(s)
        if verts.shape != (4, 3):
            print(f"[WARN] Cell with wrong shape: {verts.shape}")
            continue

        poly = Poly3DCollection([verts], facecolors='cyan', alpha=0.5, edgecolors='b')
        ax2.add_collection3d(poly)

        center = np.asarray(center).flatten()
        ax2.scatter(center[0], center[1], center[2], c='b', s=8)

plt.show()