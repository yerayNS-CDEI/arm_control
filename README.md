# arm_control

Unified ROS2 package for robot arm control, path planning, and sensor integration.

## Package History

This package was created by merging four independent packages:
- **ur_arm_control** (v1.0-pre-merge) - Robot control and trajectory execution
- **robotic_arm_planner** (v1.0-pre-merge) - Path planning and base placement
- **robotic_arm_planner_interfaces** (v1.0-pre-merge) - Service interface definitions
- **oliwall_sensors** (v1.0-pre-merge) - Hyperspectral sensor integration

For historical commit history prior to the merge, see the archived repositories.

## Structure

```
arm_control/
├── control/           # Robot control nodes (from ur_arm_control)
├── planner/          # Path planning nodes (from robotic_arm_planner)
├── sensors/          # Sensor integration nodes (from oliwall_sensors)
├── srv/              # Service definitions (from robotic_arm_planner_interfaces)
├── launch/           # Launch files
├── config/           # Configuration files
├── urdf/             # Robot URDF models
└── rviz/             # RViz configuration files
```

## Building

```bash
cd /path/to/workspace
colcon build --packages-select arm_control
source install/setup.bash
```

## Usage

The package provides various nodes for different functionalities. See individual node documentation for details.
