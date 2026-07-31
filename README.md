# arm_control: Robotic Manipulation System for Wall Scanning and Inspection

A comprehensive ROS2 manipulation package for the UR10e robotic arm that provides advanced control, motion planning, sensor integration, and mobile manipulation capabilities for autonomous wall scanning and object identification applications.

## Table of Contents
- [Overview](#overview)
- [Graphical User Interface](#graphical-user-interface)
- [Prerequisites](#prerequisites)
- [Setup](#setup)
- [Launch Files](#launch-files)
  - [Main Control](#main-control-armlaunchpy)
  - [Hardware Control](#hardware-control-ur_controllaunchpy)
  - [Simulation](#simulation-ur_sim_controllaunchpy)
  - [Motion Planning](#motion-planning-arm_moveitlaunchpy)
  - [Visualization](#visualization)
- [Workflow](#workflow)
- [Key Features](#key-features)
- [Important Notes](#important-notes)
- [Troubleshooting](#troubleshooting)

## Overview

This project provides a complete robotic manipulation solution for a **UR10e collaborative robot arm** equipped with multiple sensors for advanced wall inspection and object identification tasks.

### Robot Capabilities

The system can:
1. **Scan walls** autonomously using multiple sensor modalities (hyperspectral, multispectral, RGBD, GPR)
2. **Optimize base placement** for mobile manipulation to reach target scanning positions
3. **Plan collision-free trajectories** using reachability maps and A* pathfinding
4. **Align end-effector to walls** automatically using distance sensor feedback
5. **Identify objects** in the environment using YOLO on OAK camera during ObjectID phase
6. **Execute precise trajectories** with monitoring and error handling

### Sensor Configuration

**Scanning Sensors** (mounted on end-effector):
- **Hyperspectral cameras** (VIS: 325-793 nm, NIR: 991-1707 nm) for material analysis
- **Multispectral camera** for multi-band imaging (implementation in progress)
- **RGBD camera** for 3D structure and color capture (implementation in progress)
- **Ground Penetrating Radar (GPR)** for subsurface inspection (implementation in progress)
- **Distance sensors** (3-point array: ultrasonic + VL6180X ToF) for precise wall alignment

**Environment Perception**:
- **OAK-D camera** with YOLO for object identification during ObjectID phase
- Enables autonomous object detection and environment understanding

### Deployment Modes

The package supports two operational modes:

1. **Standalone Arm Mode**: Fixed-base operation for tasks within the arm's workspace
   - Useful when mobile base is unavailable
   - Suitable for known, reachable target positions
   - Full control and planning capabilities

2. **Mobile Manipulation Mode**: Combined arm + mobile base operation (primary use case)
   - Optimal base placement computation for unreachable targets
   - Coordinated movement of base and arm
   - Extended workspace coverage for wall scanning
   - Collision avoidance with environment

## Graphical User Interface

> **🖥️ For the complete graphical control interface, see the dedicated [UI_GUIDE.md](UI_GUIDE.md)**

This package includes a comprehensive **PyQt5-based Graphical User Interface (UI)** that provides unified control over the entire robotic system. The UI integrates both arm manipulation and mobile base navigation capabilities, making it the **recommended method for operating the system**.

### Quick Start - Launch the UI

```bash
# Launch the control UI
ros2 run arm_control UI.py
```

### UI Capabilities Overview

The UI provides an integrated control interface with **4 main tabs**:

1. **Base Control** - Mobile robot navigation and mapping (from navi_wall package)
2. **Arm Control** - UR10e arm manipulation and sensor control
3. **Joint Control** - Direct joint-level control with safety constraints
4. **Full Control** - Unified mobile manipulation (arm + base together)

### Key Features

- ✅ **Launch all system components** with configurable parameters (sim/real, planner backend)
- ✅ **Robot dashboard communication** for UR10e status and control commands
- ✅ **Emergency stop** functionality with visual feedback
- ✅ **Position control** with 18+ predefined poses
- ✅ **Sensor integration** (distance sensors, hyperspectral cameras, wall alignment)
- ✅ **Mapping and navigation** integration (RTABMap, Nav2, exploration)
- ✅ **ROS2 diagnostics** (topic monitoring, controller listing, process management)
- ✅ **Terminal integration** for live command output

The UI handles all launch configurations, parameter management, and process orchestration, eliminating the need to manually execute multiple terminal commands.

**For detailed UI documentation including all controls, workflows, and features, please refer to [UI_GUIDE.md](UI_GUIDE.md).**

## Prerequisites

### Software Requirements
- ROS2 (Humble or later)
- MoveIt2 (experimental support, in testing)
- Gazebo (for simulation)
- ros2_control and controllers
- ur_robot_driver and ur_description
- Docker (to execute URSim simulations)
- Python 3.8+
- NumPy, SciPy, Matplotlib

### Hardware Requirements
- **Robot**: UR10e collaborative robot arm (6-DOF)
- **Mobile Base**: Differential drive platform (for mobile manipulation mode)
- **Sensors**: 
  - Hyperspectral cameras (VIS/NIR)
  - Distance sensors (ultrasonic, VL6180X)
  - OAK-D camera (for ObjectID)
  - Additional sensors as needed (multispectral, RGBD, GPR)

### Critical Prerequisite: Reachability Maps

> **⚠️ IMPORTANT:** The legacy planner (primary planning backend) requires **pre-computed 4D reachability maps** to function. These maps must be generated separately with codes not included in the package. The maps needed for general purpose use are provided.

**Reachability Maps Overview:**
- **Format**: 4D voxel grids (x, y, z, orientation)
- **Purpose**: Pre-computed workspace analysis for feasibility checking and path planning
- **Required Files**:
  - `reachability_map_0.05_step_20_orientations.npy` (0.05m resolution, 20 orientation bins)
  - Optional: `reachability_map_27_fused.npy` (27-robot configuration fusion)
- **Location**: Place in `resource/` directory
- **Generation**: Code for reachability map generation is not yet included in this package. Maps will be provided separately.

## Setup

### Build the Package

```bash
cd ~/ros2_ws
colcon build --packages-select arm_control
source install/setup.bash
```

### Initial Configuration

1. **Calibration**: Ensure robot calibration data is in `config/my_robot_calibration.yaml`
2. **Initial Positions**: Configure home positions in `config/initial_positions.yaml`
3. **Controllers**: Review controller parameters in `config/ur_controllers.yaml`
4. **Reachability Maps**: Place pre-computed reachability maps in `resource/` directory

---

## Launch Files

### Main Control (`arm.launch.py`)

Primary launch file that delegates to hardware or simulation control based on parameters.

#### Purpose
- Unified entry point for arm control
- Automatically selects appropriate backend (hardware/simulation)
- Configures operational mode (standalone arm or mobile manipulation)

#### Usage

**Hardware Mode:**
```bash
ros2 launch arm_control arm.launch.py sim:=false mode:=full
```

**Simulation Mode:**
```bash
ros2 launch arm_control arm.launch.py sim:=true mode:=arm
```

#### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `sim` | `false` | Enable simulation mode |
| `mode` | `full` | Operational mode: `arm` (standalone) or `full` (mobile manipulation) |
| `ur_type` | `ur10e` | UR robot model (ur10e only supported with closed-form IK) |
| `planner_backend` | `legacy` | Planning backend: `legacy` (reachability maps) or `moveit` (experimental) |

#### Examples

```bash
# Real robot - mobile manipulation mode
ros2 launch arm_control arm.launch.py sim:=false mode:=full

# Real robot - standalone arm mode
ros2 launch arm_control arm.launch.py sim:=false mode:=arm

# Simulation - testing without mobile base
ros2 launch arm_control arm.launch.py sim:=true mode:=arm

# Experimental MoveIt2 integration
ros2 launch arm_control arm.launch.py sim:=false planner_backend:=moveit
```

---

### Hardware Control (`ur_control.launch.py`)

Launch file for controlling the real UR10e robot with full ROS2 control integration. This launch file also allows to use simulation with URSim (parameter fake_hardware:=true).

#### Purpose
- Initialize communication with UR10e hardware via RTDE protocol
- Load robot description and controllers
- Start trajectory execution and monitoring services
- Launch sensor drivers and perception nodes

#### Usage

```bash
ros2 launch arm_control ur_control.launch.py mode:=full
```

#### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `mode` | `full` | `arm` (standalone) or `full` (with mobile base) |
| `robot_ip` | `192.168.1.102` | IP address of UR10e controller |
| `use_fake_hardware` | `false` | Use mock hardware interface for testing |
| `launch_rviz` | `true` | Launch RViz visualization |
| `initial_joint_controller` | `scaled_joint_trajectory_controller` | Default controller to activate |

#### What It Launches

1. **Robot Driver**: UR robot driver node (RTDE communication)
2. **Controllers**:
   - `scaled_joint_trajectory_controller` - Main trajectory execution with speed scaling
   - `joint_trajectory_controller` - Standard trajectory controller
   - `forward_position_controller` - Direct position control
   - `speed_scaling_state_broadcaster` - Safety speed scaling
   - `force_torque_sensor_broadcaster` - F/T sensor data
3. **Control Nodes**:
   - `position_sender_node` - Predefined positions service
   - `end_effector_pose_node` - TF publisher for end-effector
   - `robot_status_check` - Safety and status monitoring
4. **Planning Services**:
   - `planner_node` or `moveit_planner_node` - Motion planning
   - `optimal_base_service` - Base placement optimization (if mode=full)
   - `wall_discretization_node` - Wall scanning grid generation
5. **Sensor Nodes**:
   - Distance sensor drivers (Arduino interface)
   - Hyperspectral camera control (if available)
   - OAK-D camera for ObjectID

#### Example Usage

```bash
# Standard mobile manipulation setup
ros2 launch arm_control ur_control.launch.py mode:=full robot_ip:=192.168.1.102

# Standalone arm with custom IP
ros2 launch arm_control ur_control.launch.py mode:=arm robot_ip:=192.168.56.101

# Testing with mock hardware
ros2 launch arm_control ur_control.launch.py use_fake_hardware:=true
```

---

### Simulation (`ur_sim_control.launch.py`)

Launch Gazebo simulation environment with UR10e robot.

#### Purpose
- Test algorithms without physical hardware
- Develop and validate trajectories safely
- Simulate sensor feedback and wall scanning

#### Usage

```bash
ros2 launch arm_control ur_sim_control.launch.py mode:=full
```

#### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `mode` | `full` | `arm` or `full` mode |
| `headless` | `false` | Run Gazebo without GUI |
| `world` | `empty.world` | Gazebo world file |
| `launch_rviz` | `true` | Launch RViz visualization |

#### Examples

```bash
# Full simulation with mobile base
ros2 launch arm_control ur_sim_control.launch.py mode:=full

# Arm-only simulation, no GUI (for CI/testing)
ros2 launch arm_control ur_sim_control.launch.py mode:=arm headless:=true

# Custom world for testing
ros2 launch arm_control ur_sim_control.launch.py world:=warehouse.world
```

---

### Motion Planning (`arm_moveit.launch.py`)

Launch MoveIt2 motion planning interface (experimental).

#### Purpose
- Provide collision-aware motion planning
- Interactive manipulation in RViz
- Planning scene management

#### Usage

```bash
ros2 launch arm_control arm_moveit.launch.py
```

> **Note**: MoveIt2 integration is currently experimental and under testing. The legacy planner is recommended for production use.

---

### Visualization

#### View Robot Model (`view_ur.launch.py`)

```bash
ros2 launch arm_control view_ur.launch.py
```

Launches RViz with robot model and joint state visualization. Useful for:
- Verifying URDF/XACRO files
- Debugging transform trees
- Visualizing robot configuration

#### Collision Visualization (`collision_view_ur.launch.py`)

```bash
ros2 launch arm_control collision_view_ur.launch.py
```

Visualizes collision meshes and planning scene for debugging:
- Collision geometry
- Self-collision checking
- Environment obstacles

---

## Workflow

### Complete Wall Scanning Workflow

This is the primary use case combining navigation, base placement, and wall scanning.

#### Step 1: Navigate to Wall Area

Use the navigation system to bring the mobile manipulator near the target wall:

```bash
# Assuming navi_wall package is running for navigation
# Set navigation goal in RViz near target wall
```

#### Step 2: Launch Arm Control System

```bash
# Set ROS domain
export ROS_DOMAIN_ID=1

# Launch arm control in mobile manipulation mode
ros2 launch arm_control arm.launch.py sim:=false mode:=full
```

This starts:
- Robot hardware interface and controllers
- Legacy planner with reachability maps
- Optimal base placement service
- Wall discretization service
- Sensor drivers (hyperspectral, distance sensors)

#### Step 3: Define Wall to Scan

Define the wall geometry and scanning parameters. The system will discretize the wall into a grid:

```bash
# Service call to discretize wall (example)
ros2 service call /wall_discretization arm_control/srv/ComputeWallDiscretization \
  "{wall_width: 2.0, wall_height: 2.0, cell_size: 0.1}"
```

The wall discretization node generates:
- Uniform grid of target scanning positions
- End-effector poses for each cell
- Coverage map for sensor footprint

#### Step 4: Compute Optimal Base Positions

For each scanning position, compute optimal mobile base placement:

```bash
# Service call to optimal base service
ros2 service call /optimal_base arm_control/srv/OptimalBase \
  "{target_pose: {...}, consider_obstacles: true}"
```

The optimal base service:
- Uses 4D reachability maps to check feasibility
- Finds closest valid base position for unreachable targets
- Considers obstacles and costmap for collision avoidance
- Returns optimal base pose and corresponding joint configuration

#### Step 5: Execute Scanning Sequence

For each wall cell:

1. **Navigate base** to optimal position (using navi_wall)
2. **Plan arm trajectory** to scanning pose using legacy planner
3. **Execute alignment**: Run `arduino_sensors` and `align_ee_to_wall` for precise positioning
4. **Capture data**: Trigger hyperspectral/multispectral/RGBD/GPR sensors
5. **Move to next** cell and repeat

#### Automatic Wall Alignment (`align_ee_to_wall`)

The alignment node uses three distance sensors on the end-effector to:
- **Measure surface orientation**: Detect if end-effector is tilted relative to wall
- **Compute correction**: Calculate angular adjustment to align parallel to wall
- **Maintain distance**: Keep optimal sensor-to-surface distance (adjustable)
- **Real-time feedback**: Continuously adjust position until alignment threshold met

```bash
# Alignment happens automatically during scanning, or can be triggered manually
ros2 service call /align_to_wall std_srvs/srv/Trigger
```

**Alignment Process:**
1. Read distance sensor array (3 points across end-effector)
2. Calculate plane orientation from distance differences
3. Compute rotation correction (roll/pitch)
4. Compute translation correction (distance from wall)
5. Apply corrections via small trajectory adjustments
6. Iterate until alignment threshold achieved (<1mm distance error, <1° angular error)

#### Step 6: Data Processing

After scanning:
- Hyperspectral data logged to CSV files with timestamps
- Point clouds merged for 3D reconstruction
- GPR data processed for subsurface analysis
- Results stored for inspection and analysis

---

### Object Identification Workflow

The ObjectID phase uses the OAK-D camera with YOLO for environment perception.

#### Purpose
- Identify and locate objects in the environment
- Build semantic map for task planning
- Detect obstacles or features of interest

#### Usage

```bash
# Launch arm control
ros2 launch arm_control arm.launch.py sim:=false mode:=full

# Move arm to predefined scanning positions
ros2 service call /send_position arm_control/srv/SendPosition "{position_name: 'front'}"
ros2 service call /send_position arm_control/srv/SendPosition "{position_name: 'left'}"
ros2 service call /send_position arm_control/srv/SendPosition "{position_name: 'right'}"

# YOLO detection runs automatically on OAK-D camera stream
# Detected objects published to /detected_objects topic
```

**Predefined Positions for ObjectID:**
- `front` - Forward-facing for frontal view
- `left` - Rotated left for side coverage
- `right` - Rotated right for opposite side
- `up` - Elevated view for overhead objects
- `down` - Downward view for ground-level objects

---

### Basic Arm Control Workflow

For simple manipulation tasks without wall scanning.

#### Sending Predefined Positions

```bash
# Launch arm control
ros2 launch arm_control arm.launch.py sim:=false mode:=arm

# Send robot to predefined positions
ros2 service call /send_position arm_control/srv/SendPosition "{position_name: 'folded'}"
ros2 service call /send_position arm_control/srv/SendPosition "{position_name: 'unfolded'}"
ros2 service call /send_position arm_control/srv/SendPosition "{position_name: 'home'}"
```

**Available Predefined Positions:**
- `folded` - Compact configuration for transport
- `unfolded` - Extended configuration
- `home` - Safe home position
- `front`, `left`, `right`, `up`, `down` - Environment scanning positions

#### Custom Trajectory Execution

```bash
# Use the publisher_joint_trajectory_planned node
ros2 run arm_control publisher_joint_trajectory_planned

# Or use MoveIt2 (if enabled)
# Set target in RViz MoveIt panel and plan/execute
```

---

### Simulation Testing Workflow

```bash
# Step 1: Launch simulation
ros2 launch arm_control arm.launch.py sim:=true mode:=full

# Step 2: Test position control
ros2 service call /send_position arm_control/srv/SendPosition "{position_name: 'home'}"

# Step 3: Test wall discretization
ros2 service call /wall_discretization arm_control/srv/ComputeWallDiscretization \
  "{wall_width: 1.5, wall_height: 2.0, cell_size: 0.15}"

# Step 4: Visualize in RViz
# Check /wall_markers topic for discretization visualization
# Check /planned_path markers for trajectory visualization

# Step 5: Test sensor simulation
# Simulated distance sensors will provide feedback in /distance_sensors topic
```

---

## Key Features

### Motion Planning

**Legacy Planner (Recommended)**:
- ✅ Uses pre-computed 4D reachability maps for fast feasibility checking
- ✅ Closed-form inverse kinematics for UR10e (specifically optimized)
- ✅ A* 3D path planning with obstacle avoidance
- ✅ Optimized for wall scanning trajectories
- ✅ Collision checking with environment and self-collision

**MoveIt2 Integration (Experimental)**:
- ⚠️ Currently in testing phase
- ⚠️ OMPL planners (RRT, PRM, etc.)
- ⚠️ Interactive planning via RViz

### Sensor Integration

**Hyperspectral Imaging**:
- VIS camera: 325-793 nm (visible spectrum)
- NIR camera: 991-1707 nm (near-infrared spectrum)
- Configurable exposure, gain, and integration time
- CSV data logging with timestamps
- Material classification and analysis

**Distance Sensing**:
- 3-point sensor array for orientation detection
- Ultrasonic + VL6180X time-of-flight sensors
- Real-time alignment feedback
- Simulator support for testing

**OAK-D Camera**:
- YOLO-based object detection
- Environment perception for ObjectID phase
- RGB-D data for 3D understanding

### Mobile Manipulation

**Optimal Base Placement**:
- ✅ Computes best mobile base position to reach any target end-effector pose
- ✅ Uses 4D reachability maps for workspace analysis
- ✅ Obstacle avoidance via costmap integration
- ✅ Efficient grid-based search algorithm
- ✅ Returns both base pose and corresponding joint configuration

**Workspace Extension**:
- Combines arm reach with mobile base mobility
- Full room coverage for wall scanning
- Coordinated arm-base motion

### Wall Scanning System

- ✅ Automatic wall discretization into uniform grid
- ✅ Sensor coverage computation
- ✅ Collision-free trajectory generation
- ✅ Precise end-effector alignment to wall surface
- ✅ Multi-modal data capture (hyperspectral, multispectral, RGBD, GPR)
- ✅ Systematic scanning with full coverage guarantee

---

## Important Notes

### Reachability Maps Requirement

> **Critical**: The legacy planner cannot function without pre-computed reachability maps. Ensure these files are in the `resource/` directory before launching:
> - `reachability_map_0.05_step_20_orientations.npy`
> - Optional: `reachability_map_27_fused.npy`
>
> Code for generating reachability maps may be integrated in future releases. For now, they must be provided separately.

### UR10e Specific

The closed-form inverse kinematics algorithm is **specifically designed for UR10e**. Other UR models (UR3, UR5e, UR16e) are not supported with the legacy planner. MoveIt2 integration may provide support for other models in the future.

### Safety Considerations

Always observe these safety practices:
- Ensure emergency stop is accessible and functional
- Test new trajectories in simulation first
- Monitor robot during autonomous operation
- Keep workspace clear of personnel during scanning
- Verify collision geometries are accurate
- Use reduced speed scaling for initial tests

### Known Limitations

- **Freedrive mode**: Currently not functional, do not use
- **MoveIt2**: Experimental, may have stability issues
- **GPR integration**: Implementation in progress
- **Multispectral camera**: Implementation in progress
- **RGBD camera**: Implementation in progress

---

## Additional Resources

### Directory Structure

```
arm_control/
├── config/              # Configuration files
│   ├── ur_controllers.yaml           # ROS2 controller definitions
│   ├── initial_positions.yaml        # Predefined pose library
│   ├── my_robot_calibration.yaml     # Robot-specific calibration
│   └── moveit/                       # MoveIt2 configuration
├── control/             # Control nodes
│   ├── position_sender_node.py       # Position service
│   ├── end_effector_pose_node.py     # TF broadcaster
│   ├── exhaustive_scan_node.py       # Scanning orchestration
│   ├── publisher_joint_trajectory_planned.py  # Trajectory execution
│   └── robot_status_check.py         # Safety monitoring
├── planner/             # Planning nodes
│   ├── planner_node.py               # Legacy planner (reachability maps)
│   ├── moveit_planner_node.py        # MoveIt2 interface
│   ├── optimal_base_service.py       # Base placement optimization
│   ├── wall_discretization_node.py   # Wall scanning grid
│   ├── planner_lib/                  # Planning algorithms (IK, A*)
│   └── rm4d_lib/                     # Reachability map library
├── sensors/             # Sensor drivers
│   ├── hyperspectral_node.py         # Hyperspectral camera control
│   ├── align_ee_to_wall.py           # Automatic alignment
│   └── sensors_distance_orientation.py  # Distance sensor fusion
├── launch/              # Launch files
├── urdf/                # Robot description files
├── srdf/                # MoveIt semantic description
├── srv/                 # Custom service definitions
├── resource/            # Reachability maps and data
└── rviz/                # RViz configuration files
```

### Key Topics

**Control Topics**:
- `/scaled_joint_trajectory_controller/joint_trajectory` - Trajectory commands
- `/joint_states` - Current joint positions
- `/cmd_vel` - Mobile base velocity (if mode=full)

**Planning Topics**:
- `/planned_path` - Visualization of planned trajectories
- `/reachability_markers` - Reachability map visualization
- `/wall_markers` - Wall discretization grid

**Sensor Topics**:
- `/hyperspectral/vis/data` - VIS hyperspectral data
- `/hyperspectral/nir/data` - NIR hyperspectral data
- `/distance_sensors` - 3-point distance array
- `/oak/detected_objects` - YOLO detections

**Status Topics**:
- `/robot_status` - Safety and operational status
- `/end_effector_pose` - Current EE pose

### Key Services

| Service | Type | Description |
|---------|------|-------------|
| `/send_position` | `SendPosition` | Send robot to predefined pose |
| `/optimal_base` | `OptimalBase` | Compute optimal base position |
| `/wall_discretization` | `ComputeWallDiscretization` | Generate scanning grid |
| `/align_to_wall` | `Trigger` | Execute wall alignment |
| `/hyperspectral_command` | `HyperspectralCommand` | Control hyperspectral camera |

### Configuration Files

**Controller Configuration** (`config/ur_controllers.yaml`):
- Defines available controllers (trajectory, position, force, velocity)
- Sets controller parameters (gains, limits, tolerances)
- Configure speed scaling and safety limits

**Initial Positions** (`config/initial_positions.yaml`):
- Library of predefined joint configurations
- Named poses for common operations
- Safe transit poses

**Robot Calibration** (`config/my_robot_calibration.yaml`):
- DH parameter corrections
- Kinematic calibration offsets
- Tool center point (TCP) definition

---

## Troubleshooting

### Common Issues

**Issue**: "Reachability map file not found"
```
Solution: Ensure reachability map .npy files are in the resource/ directory.
The legacy planner requires these pre-computed maps to function.
```

**Issue**: "IK solution not found"
```
Solution: 
1. Check if target pose is within robot workspace
2. Verify target orientation is feasible
3. Try optimal_base_service to find better base position
4. Check for collisions in target configuration
```

**Issue**: "Controller failed to load"
```
Solution:
1. Verify robot IP address is correct
2. Check robot is powered on and in remote control mode
3. Ensure no other programs are connected to robot
4. Check network connectivity (ping robot IP)
```

**Issue**: "Trajectory execution aborted"
```
Solution:
1. Check robot status for safety violations
2. Verify speed scaling is not at 0%
3. Check for unexpected collisions
4. Review trajectory in simulation first
```

**Issue**: "Distance sensors returning invalid data"
```
Solution:
1. Check Arduino connection (USB or serial)
2. Verify sensor wiring and power
3. Test sensors individually
4. Check for sensor obstruction or damage
```

**Issue**: "Wall alignment not converging"
```
Solution:
1. Verify all 3 distance sensors are functional
2. Ensure wall surface is flat and smooth
3. Check initial pose is approximately correct (within ~10cm, ~10°)
4. Adjust alignment tolerances if needed
5. Verify end-effector is not too far from wall (>50cm)
```

**Issue**: "MoveIt2 planning fails"
```
Note: MoveIt2 integration is experimental. Consider using legacy planner instead.
Solution:
1. Check planning scene is loaded correctly
2. Verify collision objects are defined
3. Increase planning time limit
4. Try different planner (RRT vs PRM)
```

---

**Package Maintainer**: [Contact information]  
**License**: [License type]  
**ROS2 Version**: Humble  
**Last Updated**: March 2026
