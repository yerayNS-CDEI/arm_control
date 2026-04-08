# arm_control UI: Comprehensive User Guide

A complete guide to the PyQt5-based graphical user interface for controlling the UR10e robotic manipulator and mobile base platform.

## Table of Contents
- [Overview](#overview)
- [Getting Started](#getting-started)
- [UI Layout and Navigation](#ui-layout-and-navigation)
- [Tab 1: Base Control](#tab-1-base-control)
- [Tab 2: Arm Control](#tab-2-arm-control)
- [Tab 3: Joint Control](#tab-3-joint-control)
- [Tab 4: Full Control](#tab-4-full-control)
- [Common Features](#common-features)
- [Configuration Guide](#configuration-guide)
- [Workflow Examples](#workflow-examples)
- [Troubleshooting](#troubleshooting)
- [Safety and Best Practices](#safety-and-best-practices)

---

## Overview

The arm_control UI is a comprehensive graphical interface that integrates:
- UR10e robotic arm control and planning
- Mobile base navigation and mapping (navi_wall package)
- Sensor management (hyperspectral cameras, distance sensors)
- Robot dashboard communication
- ROS2 diagnostics and process management

The UI eliminates the need to manually execute multiple terminal commands by providing a unified control interface with intelligent configuration management.

**Key Benefits:**
- Single application for all robot operations
- Real-time status monitoring and logging
- Context-aware parameter configuration
- Process lifecycle management
- Emergency stop functionality with visual feedback

---

## Getting Started

### Installation Requirements

**Software Dependencies:**
- ROS2 (Humble or later)
- PyQt5
- QTermWidget (for terminal integration)

```bash
# Install PyQt5 and terminal widget
sudo apt install python3-pyqt5 libqtermwidget5-0

# Ensure arm_control and navi_wall packages are built
cd ~/ros2_ws
colcon build --packages-select arm_control navi_wall
source install/setup.bash
```

### Launching the UI

```bash
# Set ROS domain ID first
export ROS_DOMAIN_ID=1

# Launch the UI
ros2 run arm_control UI.py
```

The UI window will open with 4 tabs: Base Control, Arm Control, Joint Control, and Full Control.

**[SCREENSHOT PLACEHOLDER: UI main window showing all 4 tabs]**

---

## UI Layout and Navigation

### Tab Structure

The UI is organized into **4 main tabs**, each designed for specific operational modes:

| Tab | Purpose | Used When |
|-----|---------|-----------|
| **Base Control** | Mobile robot navigation only | Operating base without arm |
| **Arm Control** | Manipulator control only | Operating arm without mobile base |
| **Joint Control** | Direct joint-level control | Fine-tuning joint positions, teaching |
| **Full Control** | Unified mobile manipulation | Primary use case - arm + base together |

### Tab Mutual Exclusivity

**Important:** Only one operational mode can be active at a time. When launching processes in one tab:
- Other operational tabs become **disabled** (grayed out)
- This prevents conflicting robot control states
- Stop all processes in the current tab to unlock others

Example: Starting "Arm Control" disables "Base Control" and "Full Control" tabs.

**[SCREENSHOT PLACEHOLDER: UI showing disabled tabs when one mode is active]**

### Status Text Areas

Each tab includes a **status text display** showing:
- Command execution output (ANSI color-coded)
- ROS node messages and logs
- Process state changes
- Error and warning messages

**Features:**
- **Search**: Find text in status logs (see search box in each tab)
- **Clear/Restore**: Clear status text and restore it later
- **Auto-scroll**: Automatically scrolls to bottom unless you're viewing older content

---

## Tab 1: Base Control

**Purpose:** Control the mobile robot platform for navigation, mapping, and exploration.

**[SCREENSHOT PLACEHOLDER: Base Control tab overview]**

### Configuration Options

Located at the top of the tab:

| Option | Values | Description |
|--------|--------|-------------|
| **Simulation** | `false`, `true` | Use Gazebo simulation instead of real robot |
| **Controller Type** | `diff`, `omni` | Differential drive or omnidirectional controller |
| **Headless** | `false`, `true` | Run Gazebo without GUI (simulation only) |

**[SCREENSHOT PLACEHOLDER: Base Control configuration checkboxes]**

### Mapping and Localization Box

Create and use 3D maps with RTABMap SLAM.

#### Buttons:

**Start Mapping**
- Launches: `ros2 launch navi_wall mapping_3d.launch.py`
- Parameters: Uses configured simulation mode, controller type, headless setting
- Purpose: Create 3D maps with visual-LiDAR fusion
- Usage: Click to start, teleoperate robot with joystick, click "Stop Mapping" when done
- Map saved to: `maps/rtabmap.db`

**Start Localization**
- Launches: `ros2 launch navi_wall move_robot.launch.py`
- Purpose: Localize within existing map for autonomous navigation
- Prerequisites: Must have created a map first
- Note: Requires Nav2 to be launched separately

**View Map**
- Launches: `rtabmap-databaseViewer rtabmap.db`
- Purpose: Visualize and analyze RTABMap database
- Shows: 3D point cloud map, graph structure, loop closures

**[SCREENSHOT PLACEHOLDER: Mapping and Localization box]**

### Navigation and Exploration Box

Navigate autonomously within mapped environments.

#### Buttons:

**Launch Nav2**
- Launches: `ros2 launch navi_wall navigation_launch.py`
- Purpose: Start Nav2 navigation stack for autonomous navigation
- Prerequisites: Localization must be running
- Usage: Set goals in RViz using "Nav2 Goal" tool

**Launch Exploration (explore_lite)**
- Launches: `ros2 run navi_wall explore`
- Purpose: Autonomous frontier-based exploration
- Prerequisites: Nav2 must be running
- Usage: Robot will automatically explore unmapped areas

**[SCREENSHOT PLACEHOLDER: Navigation and Exploration box]**

### Troubleshooting Box

Debug and monitor system state.

#### Buttons:

**List ROS2 Processes**
- Command: `ps aux | grep -E 'ros2|robot'`
- Purpose: Show all running ROS2-related processes
- Output: Process list displayed in status text
- Usage: Check if nodes are running or identify zombie processes

**List Controllers**
- Purpose: Query all ros2_control controller managers
- Shows: Active and inactive controllers
- Usage: Verify controller state, troubleshoot control issues

**Start RQT**
- Launches: `rqt`
- Purpose: Open RQT for advanced debugging and visualization
- Plugins: Access topic monitoring, service calls, parameter tuning, etc.

**[SCREENSHOT PLACEHOLDER: Troubleshooting box]**

### Status Display and Search

Located on the right side of the tab.

**[SCREENSHOT PLACEHOLDER: Status text area with search controls]**

#### Search Box:
- Type text to search, press **Enter** to find next occurrence
- Navigate: Use **◀** (previous) and **▶** (next) buttons
- Highlighting: Found text is highlighted in yellow
- Wrap-around: Searches cycle from end to beginning

#### Clear and Restore:
- **Clear**: Temporarily hide all status text
- **Restore**: Bring back previously cleared text
- Usage: Clear clutter when status gets long, restore if you need to review

### Terminal Widget

The terminal on the left shows live process output with proper formatting and colors.

---

## Tab 2: Arm Control

**Purpose:** Control the UR10e robotic arm for manipulation, scanning, and positioning tasks.

**[SCREENSHOT PLACEHOLDER: Arm Control tab overview]**

### Configuration Options

Located at the top of the tab:

| Option | Values | Description |
|--------|--------|-------------|
| **Simulation** | `false`, `true` | Use simulation (Gazebo or URSim) instead of real robot |
| **Planner Backend** | `legacy`, `moveit` | Motion planning backend (legacy=reachability maps, moveit=experimental) |
| **URSim** | `false`, `true` | Use URSim (Universal Robots simulator in Docker) |

**Important Interactions:**
- When **Planner Backend = moveit**: URSim is **locked to true**, Joint Control tab is **disabled**
- When **Simulation = true**: URSim option becomes available
- URSim uses robot IP: `192.168.56.101` (localhost Docker)
- Real robot uses IP: `192.168.1.102`

**[SCREENSHOT PLACEHOLDER: Arm Control configuration options]**

### Control Box

Main arm system control.

#### Buttons:

**Start Arm**
- Launches: `ros2 launch arm_control arm.launch.py`
- Parameters: 
  - `sim:=<true/false>`
  - `robot_ip:=<IP_address>` (auto-configured based on sim/URSim settings)
  - `planner_backend:=<legacy/moveit>`
  - `mode:=arm` (standalone arm mode)
- Purpose: Initialize arm hardware/simulation, controllers, and planning services
- Button state: Changes to "Stop Arm" when running (green color)

**Launch RQT Joint Controller**
- Launches: `ros2 run rqt_joint_trajectory_controller`
- Purpose: Alternative joint control interface
- Usage: Manual control of individual joints via sliders

**Reset Planner**
- Command: `ros2 topic pub --once /planner/reset std_msgs/msg/Bool "{data: true}"`
- Purpose: Reset legacy planner state (clear obstacles, reset constraints)
- Note: **Disabled** when Planner Backend = moveit

**List Controllers**
- Purpose: Query all available ros2_control controller managers
- Shows: Active and available controllers for the arm
- Usage: Verify controller state, check which controllers are loaded

**[SCREENSHOT PLACEHOLDER: Control box]**

### Robot Initialization Box (Dashboard Commands)

Direct communication with UR10e robot dashboard for status monitoring and control.

**[SCREENSHOT PLACEHOLDER: Robot Initialization box]**

#### Status Commands Dropdown:

Select and send status query commands to the robot:

| Command | Returns |
|---------|---------|
| `robotmode` | Current robot mode (RUNNING, IDLE, etc.) |
| `safetystatus` | Safety system status |
| `programState` | Program execution state (PLAYING, STOPPED, PAUSED) |
| `running` | Whether program is currently running (true/false) |
| `get loaded program` | Name of currently loaded program |
| `is in remote control` | Whether robot is in remote control mode (true/false) |

**Buttons:**
- **Send Status Command**: Send selected status query
- **Send All Status Commands**: Send all 6 status commands sequentially

#### Control Commands Dropdown:

Send control commands to the robot:

| Command | Action |
|---------|--------|
| `power on` | Power up robot motors |
| `power off` | Power down motors |
| `brake release` | Release electromagnetic brakes |
| `play` | Start/resume program execution |
| `pause` | Pause program execution |
| `stop` | Emergency stop (hard stop) |
| `shutdown` | Complete system shutdown |
| `close popup` | Close any open popup dialogs on teach pendant |
| `restart safety` | Restart safety system |
| `load Test_external_control.urp` | Load external control program |
| `close safety popup` | Close safety-related popup dialogs |
| `unlock protective stop` | Unlock protective stop after safety event |

**Buttons:**
- **Send Control Command**: Send selected control command

**Connection Details:**
- Host: Determined by Simulation and URSim settings
- Port: 29999 (standard UR dashboard port)
- Timeout: 5 seconds
- Output: Command responses displayed in status text

**Note:** This box is **disabled** when Simulation=true AND URSim=false (Gazebo simulation has no dashboard).

### Position Sender

Send robot to predefined positions quickly.

**[SCREENSHOT PLACEHOLDER: Position Sender dropdown and button]**

#### Available Positions:

| Position | Description |
|----------|-------------|
| `folded` | Compact configuration for transport |
| `unfolded` | Extended configuration |
| `up` | Elevated end-effector position |
| `down` | Lowered end-effector position |
| `front` | Forward-facing for frontal scanning |
| `left` | Rotated left for side coverage |
| `right` | Rotated right for opposite side |
| `one`, `two`, `three`, `four`, `five`, `six` | Numbered waypoints |
| `initial` | Initial/home position |
| `under`, `under1`, `under2` | Positions for scanning underside surfaces |
| `p1` | Custom position 1 |
| `custom` | Custom position (user-defined) |

**Button:**
- **Send Position**: Service call to `/send_position` with selected position name
- Service waits for trajectory completion or timeout

**Usage:**
1. Select position from dropdown
2. Click "Send Position"
3. Robot plans and executes trajectory
4. Status text shows success or error messages

### Sensors Box

Control sensor drivers and alignment systems.

**[SCREENSHOT PLACEHOLDER: Sensors box]**

#### Buttons:

**Start Distance Sensors**
- Real robot: `ros2 run arm_control arduino_sensors`
- Simulation: `ros2 run arm_control arduino_sensors_sim`
- Purpose: Start 3-point distance sensor array driver (ultrasonic + VL6180X)
- Topic: Publishes to `/arm/distance_sensors` (Float32MultiArray)
- Usage: Must be running before using wall alignment

**Start Align EE to Wall**
- Command: `ros2 run arm_control align_ee_to_wall`
- Purpose: Automatic end-effector alignment to wall surface
- Prerequisites: Distance sensors must be running
- Algorithm: 
  - Reads 3-point distance measurements
  - Computes orientation correction (roll/pitch)
  - Computes distance correction (Z-axis)
  - Adjusts end-effector pose iteratively until aligned
- Thresholds: <1mm distance error, <1° angular error

### Emergency Stop

Critical safety control (always visible at bottom of arm section).

**[SCREENSHOT PLACEHOLDER: Emergency stop button in normal and active states]**

**Button States:**
- **Normal (RED)**: "EMERGENCY STOP (Click to Activate)"
- **Active (DARK RED)**: "EMERGENCY STOP ACTIVE (Click to Release)"

**Activation (Click when normal):**
1. Publishes `Bool(data=true)` to emergency stop topic
2. Cancels active trajectory action goals
3. Sends `stop` command to robot dashboard
4. Status: "⚠ EMERGENCY STOP - Published stop signal"

**Release (Click when active):**
1. Publishes `Bool(data=false)` to emergency stop topic
2. Sends `close safety popup` to robot
3. Sends `unlock protective stop` to robot
4. Sends `play` to resume program
5. Status: "✓ EMERGENCY STOP RELEASED - Published release signal"

**Topic Selection (context-aware):**
- Planner=moveit → `/arm/emergency_stop`
- Planner=legacy → `/emergency_stop`

### Status Display and Search

Same functionality as Base Control tab - see [Status Display and Search](#status-display-and-search).

---

## Tab 3: Joint Control

**Purpose:** Direct control of individual joint positions with safety constraints.

**[SCREENSHOT PLACEHOLDER: Joint Control tab overview]**

### Configuration

Joint Control **inherits settings** from the active planner context:
- If last used Arm Control tab → uses Arm settings
- If last used Full Control tab → uses Full Control settings

### Joint Position Control

**[SCREENSHOT PLACEHOLDER: Joint sliders and input fields]**

#### Joint Input Methods:

For each of the 6 joints (`arm_shoulder_pan_joint`, `arm_shoulder_lift_joint`, `arm_elbow_joint`, `arm_wrist_1_joint`, `arm_wrist_2_joint`, `arm_wrist_3_joint`):

1. **Slider** (disabled until joint positions read)
   - Range: -6.28 to +6.28 radians
   - Visual feedback on current position

2. **Input Field** (disabled until joint positions read)
   - Accepts numeric values in radians
   - Validation: Must be within ±6.28 rad
   - Type value and press Enter to update slider

3. **Label**
   - Shows joint name and current value
   - Updates in real-time from joint state topic

#### Time from Start Input:

- Sets trajectory duration
- Range: 0.1 to 60.0 seconds
- Default: 1.0 second
- Usage: Longer time = slower, smoother motion

### Control Buttons

**[SCREENSHOT PLACEHOLDER: Joint control buttons]**

**Read Current Joint Positions**
- Subscribes to joint state topic (context-dependent: `/joint_states` or `/arm/joint_states`)
- Reads current robot configuration
- **Enables** sliders and input fields only after successful read
- Validates that all 6 expected joints are present
- Error handling: Shows warning if joints are missing

**Publish Joint Trajectory**
- Publishes joint trajectory to planning topic
- **Safety Constraint**: Only **one joint can change at a time**
- Tolerance: 0.015 radians (other joints must be within this tolerance of original position)
- Topic: `/planned_trajectory` or `/arm/planned_trajectory`
- Validation: 
  - Checks that exactly 1 joint changed
  - Warns if multiple joints changed (blocks publication)
  - Shows which joints changed by how much
- Success: "✓ Trajectory published successfully"

**Reset Planner**
- Same as in Arm Control tab
- Publishes reset message to planner

### Safety Mechanisms

**Single-Joint Constraint:**
- **Why**: Prevents dangerous multi-joint movements in manual control
- **Enforcement**: Software check before publishing trajectory
- **Override**: None - this is a hard safety constraint
- **Workaround**: Move joints one at a time, read position between moves

**Position Validation:**
- All joints checked against limits (±6.28 rad for UR10e)
- Out-of-range values rejected with error message

### Terminal and Status

- **Left**: QTermWidget for live process output
- **Right**: Status text with Clear button
- Shows trajectory publication results and errors

**[SCREENSHOT PLACEHOLDER: Joint Control terminal and status display]**

---

## Tab 4: Full Control

**Purpose:** Unified control of mobile manipulation system (arm + base together). This is the primary operational mode for wall scanning applications.

**[SCREENSHOT PLACEHOLDER: Full Control tab overview]**

### Configuration Options

Located at the top of the tab:

| Option | Values | Description |
|--------|--------|-------------|
| **Simulation** | `false`, `true` | Use simulation instead of real hardware |
| **Controller Type** | `diff`, `omni` | Mobile base controller type |
| **Planner Backend** | `legacy`, `moveit` | Arm motion planning backend |
| **Hybrid Sim (URSim)** | `false`, `true` | Use URSim for arm in simulation |
| **Headless** | `false`, `true` | Run Gazebo without GUI |

**Important Interactions:**
- **Planner Backend = moveit** → Hybrid Sim **locked to true**
- **Simulation = true AND Hybrid Sim = true** → Headless option **hidden** (URSim always has UI)
- **Simulation = true AND Hybrid Sim = false** → Headless option **shown**
- **Hybrid Sim = true** → Robot Initialization box **enabled**
- **Hybrid Sim = false** → Robot Initialization box **disabled**

**[SCREENSHOT PLACEHOLDER: Full Control configuration options showing all checkboxes]**

### Namespace Behavior

When **Simulation=true AND Hybrid_Sim=true**, the UI uses namespaced topics/services:
- Joint states: `/arm/joint_states`
- Emergency stop: `/arm/emergency_stop`
- Send position: `/arm/send_position`
- Planned trajectory: `/arm/planned_trajectory`

Otherwise, uses root namespace (no `/arm/` prefix).

### Robot Initialization Box

Identical to Arm Control tab - see [Robot Initialization Box](#robot-initialization-box-dashboard-commands).

**[SCREENSHOT PLACEHOLDER: Robot Initialization box in Full Control]**

### Mapping, Localization, Navigation, Exploration Box

Combined control integrating both base and arm operations.

**[SCREENSHOT PLACEHOLDER: Mapping/Navigation box in Full Control]**

#### Buttons:

**Start Mapping**
- Launches: `ros2 launch navi_wall mapping_3d.launch.py`
- Parameters: `mode:=full` (includes mobile base + arm)
- Purpose: Create 3D maps with the full mobile manipulator
- Usage: Arm can be positioned during mapping for better sensor coverage

**Start Localization**
- Launches: `ros2 launch navi_wall move_robot.launch.py`
- Parameters: `mode:=full`, `planner_backend:<legacy/moveit>`
- Purpose: Localize mobile manipulator in existing map
- Integration: Arm maintains position while base moves

**Launch Nav2**
- Same as Base Control tab
- Purpose: Enable autonomous navigation to scan positions

**Launch Exploration (explore_lite)**
- Same as Base Control tab
- Purpose: Autonomous map building

**Send Position**
- Same as Arm Control tab
- Service: Namespaced or root based on configuration
- Purpose: Move arm to predefined positions

**Reset Planner**
- Same as Arm Control tab
- Topic: Namespaced or root based on configuration

**EMERGENCY STOP (Click to Activate)**
- Same as Arm Control tab
- Synchronized with Arm Control emergency stop button
- Topic selection: Context-aware based on Hybrid Sim setting

### Sensors Box

**[SCREENSHOT PLACEHOLDER: Sensors box with oliwall.png background]**

Background image: Shows `oliwall.png` (robot illustration, 250x200px)

#### Button:

**Start Align EE to Wall**
- Same as Arm Control tab
- Process key: `full_align_ee_to_wall` (separate from arm tab instance)

### Troubleshooting Box

Advanced debugging and monitoring tools.

**[SCREENSHOT PLACEHOLDER: Troubleshooting box in Full Control]**

#### Buttons:

**View map**
- Launches: `rtabmap-databaseViewer rtabmap.db`
- Same as Base Control tab

**List Controllers**
- Queries controller managers for both base and arm
- Shows all active controllers in the system

**Start RQT**
- Launches: `rqt`
- Useful for visualizing both base and arm topics simultaneously

**List ROS2 Processes**
- Command: `ps aux | grep -E 'ros2|robot'`
- Populates process dropdown for kill functionality

### ROS2 Topics Section

Monitor and analyze ROS2 topics in real-time.

**[SCREENSHOT PLACEHOLDER: ROS2 Topics section with dropdown and buttons]**

#### Topic Selector:
- Dropdown (editable/searchable)
- Auto-populated by clicking "Refresh"
- Manual entry supported for custom topics

#### Buttons:

**Refresh**
- Command: `ros2 topic list`
- Updates dropdown with all available topics
- Usage: Click after launching new nodes

**Bandwidth**
- Command: `timeout 4 ros2 topic bw <topic>`
- Measures: Message bandwidth in MB/s
- Output: "Message size mean: X MB" displayed in info widget
- Timeout: 4 seconds (auto-kills process)

**Frequency**
- Command: `timeout 4 ros2 topic hz <topic>`
- Measures: Publishing rate in Hz
- Output: "Average rate: X Hz" displayed in info widget
- Timeout: 4 seconds

**Echo Once**
- Command: `timeout 10 ros2 topic echo --once <topic>`
- Shows: Full message content
- Output: Displayed in status text (can be lengthy)
- Timeout: 10 seconds

**Info Display:**
- Shows bandwidth or frequency results
- Max height: 60px
- Compact display for quick reference

**[SCREENSHOT PLACEHOLDER: Topic info display showing bandwidth/frequency result]**

### Process Management

Kill frozen or unwanted ROS2 processes.

**[SCREENSHOT PLACEHOLDER: Process management dropdown and kill button]**

#### Workflow:

1. **List ROS2 Processes** → Populates process dropdown
2. **Select process** from dropdown
   - Displays as: "PID: XXXX"
   - Hover tooltip: Shows full command line
3. **Kill Process** → Executes `kill -9 <PID>`
   - Confirmation dialog: "Are you sure you want to kill process XXXX?"
   - On success: Refreshes process list automatically
   - On failure: Shows error message

**Usage:**
- Find zombie processes preventing system shutdown
- Kill stuck nodes
- Clean up processes after crashes

**Warning:** Use carefully - killing critical processes may destabilize the system.

### Status Display and Search

Same functionality as other tabs, with enhanced search capabilities.

**[SCREENSHOT PLACEHOLDER: Full Control status text with search controls]**

---

## Common Features

Features available across multiple tabs.

### Search Functionality

All status text areas include search capabilities.

**[SCREENSHOT PLACEHOLDER: Search box with buttons highlighted]**

**Usage:**
1. Type search term in search box
2. Press **Enter** or click **▶** (next)
3. Use **◀** for previous occurrence
4. Found text highlighted in yellow
5. Wrap-around: Automatically cycles through results

**Tips:**
- Case-insensitive search
- Searches entire status history (including scrolled content)
- HTML formatting preserved in status text

### Clear and Restore

Manage status text visibility.

**Workflow:**
1. Click **Clear** → Status text hidden
2. **Restore** button becomes enabled
3. Click **Restore** → Previous content prepended back
4. Hidden content preserved (not deleted)

**Usage:**
- Clear clutter when status gets very long
- Restore if you need to review previous output
- Useful for focusing on new output

### Status Text Formatting

**Color Coding (ANSI to HTML):**
- Commands: Bold green with ▶ symbol
- Success: Green with ✓ symbol
- Errors: Red with ✗ symbol
- Warnings: Yellow/orange with ⚠ symbol
- Info: Various colors based on message type

**Auto-Scrolling:**
- Scrolls to bottom automatically when new content arrives
- **Preserves scroll position** if you're viewing older content
- Smart detection: Only auto-scrolls if you're already at bottom

### Process State Tracking

All launch buttons (Start Arm, Start Mapping, etc.) track process state:

**Visual Indicators:**
- **Running**: Button text changes to "Stop X", background turns green (#4CAF50)
- **Stopped**: Button text "Start X", default background color

**Process Lifecycle:**
1. Click to start → Process launched, button turns green
2. Click again to stop → Graceful shutdown sequence:
   - Send SIGINT (Ctrl+C equivalent)
   - Wait 3-5 seconds for clean shutdown
   - Send SIGTERM if still running
   - Wait 2 seconds
   - Send SIGKILL if still running
3. Button returns to normal state

**Special Handling:**
- **Mapping processes**: 5-second wait after SIGINT (allows database save)
- **Gazebo**: Force-kills all `gz sim` processes
- **Child processes**: Automatically killed with parent via `pkill -9 -P <parent_PID>`

---

## Configuration Guide

Understanding how configuration options interact.

### Simulation Modes

**[SCREENSHOT PLACEHOLDER: Configuration comparison table visualization]**

#### Arm Control - Real Hardware
```
Simulation: false
Planner Backend: legacy
URSim: N/A (hidden)
```
- Robot IP: 192.168.1.102
- Uses physical UR10e
- Robot Initialization: Enabled
- Joint Control: Enabled

#### Arm Control - Gazebo Simulation
```
Simulation: true
Planner Backend: legacy
URSim: false
```
- Gazebo simulation with UR10e model
- Robot Initialization: **Disabled** (no dashboard)
- Robot IP: 192.168.1.102 (unused in Gazebo)
- Joint Control: Enabled

#### Arm Control - URSim Simulation
```
Simulation: true
Planner Backend: legacy
URSim: true
```
- Uses Docker-based URSim
- Robot IP: 192.168.56.101 (localhost)
- Robot Initialization: Enabled
- Joint Control: Enabled
- Most realistic simulation

#### Arm Control - MoveIt Mode
```
Simulation: true (recommended)
Planner Backend: moveit
URSim: true (locked)
```
- URSim automatically enabled
- Robot Initialization: Enabled
- Joint Control: **Disabled** (MoveIt replaces it)
- Experimental - under testing

### Full Control Modes

#### Real Mobile Manipulator
```
Simulation: false
Controller Type: diff
Planner Backend: legacy
Hybrid Sim: false
```
- Both base and arm are real hardware
- Full integration mode
- Robot Initialization: Enabled

#### Hybrid Simulation (Base Real, Arm URSim)
```
Simulation: true
Controller Type: diff
Planner Backend: legacy
Hybrid Sim: true
```
- Base in Gazebo
- Arm in URSim (Docker)
- Namespace: `/arm/` prefix used
- Robot Initialization: Enabled
- Testing arm algorithms with simulated base

#### Full Gazebo Simulation
```
Simulation: true
Controller Type: diff
Planner Backend: legacy
Hybrid Sim: false
Headless: true/false
```
- Both base and arm in Gazebo
- Robot Initialization: **Disabled**
- Fastest for testing navigation
- Headless option available to hide Gazebo GUI

### Tab Enabling Logic

**When Base Control processes running:**
- Base Control tab: Active
- Arm Control tab: **Disabled**
- Joint Control tab: **Disabled**
- Full Control tab: **Disabled**

**When Arm Control "Start Arm" running:**
- Base Control tab: **Disabled**
- Arm Control tab: Active
- Joint Control tab: Active (if planner ≠ moveit)
- Full Control tab: **Disabled**

**When Full Control processes running:**
- Base Control tab: **Disabled**
- Arm Control tab: **Disabled**
- Joint Control tab: Active (if planner ≠ moveit)
- Full Control tab: Active

**Rationale:** Prevents conflicting robot control commands from multiple sources.

---

## Workflow Examples

Step-by-step guides for common tasks.

### Workflow 1: Create a Map (Base Only)

**[SCREENSHOT PLACEHOLDER: Workflow diagram for mapping]**

1. Open UI: `ros2 run arm_control UI.py`
2. Go to **Base Control** tab
3. Configure:
   - Simulation: `true` (for testing) or `false` (real robot)
   - Controller Type: `diff`
   - Headless: `false` (to see Gazebo)
4. Click **Start Mapping**
5. Wait for RViz and Gazebo to open
6. Teleoperate robot with joystick
   - Drive through all areas to map
   - Ensure good loop closures
7. Click **Stop Mapping** when complete
8. Map saved to `~/ros2_ws/src/navi_wall/maps/rtabmap.db`

**Status Indicators:**
- "Starting mapping process..."
- "RTABMap node started"
- "Loop closure detected" (periodic)

### Workflow 2: Navigate with Arm Scanning

**[SCREENSHOT PLACEHOLDER: Workflow diagram for navigation + scanning]**

1. Open UI: `ros2 run arm_control UI.py`
2. Go to **Full Control** tab
3. Configure:
   - Simulation: `false` (real robot)
   - Controller Type: `diff`
   - Planner Backend: `legacy`
   - Hybrid Sim: `false`
4. Click **Start Localization**
5. Wait for map to load in RViz
6. Click **Launch Nav2**
7. In RViz: Set navigation goal near target wall
8. While base drives:
   9. Select position: `front`
   10. Click **Send Position** (arm moves to scanning position)
11. Robot reaches goal
12. Use wall scanning workflow (see next section)

### Workflow 3: Wall Scanning with Alignment

**[SCREENSHOT PLACEHOLDER: Workflow diagram for wall scanning]**

**Prerequisites:**
- Robot positioned near target wall (via navigation or manual positioning)
- Localization running (if using Full Control)

**Steps:**

1. In **Arm Control** or **Full Control** tab
2. Check configuration matches your setup
3. If not already running: Click **Start Arm**
4. In **Sensors Box**:
   - Click **Start Distance Sensors**
   - Wait for "Distance sensors started" message
5. Position arm approximately facing wall:
   - Select position: `front` (or appropriate direction)
   - Click **Send Position**
6. In **Sensors Box**:
   - Click **Start Align EE to Wall**
   - Monitor status text for alignment progress:
     - "Distance sensor readings: [d1, d2, d3]"
     - "Orientation error: X degrees"
     - "Distance error: Y mm"
     - "Applying correction trajectory"
     - "Alignment achieved"
7. Trigger hyperspectral scan (via separate node or service - not yet integrated in UI)
8. Move to next scan position:
   - Use Position Sender or plan custom trajectory
   - Repeat alignment and scan

**Typical Alignment Sequence:**
```
▶ Starting alignment node
⏹ Distance sensors: [45.2, 46.1, 45.8] mm
🔄 Orientation error: 2.3°, Distance error: 5.4mm
🔄 Iteration 1: Orientation error: 0.8°, Distance error: 1.2mm
✓ Alignment achieved! Error <1.0mm, <1.0°
```

### Workflow 4: Object Identification

**[SCREENSHOT PLACEHOLDER: Workflow diagram for ObjectID]**

**Purpose:** Scan environment with OAK-D camera and YOLO for object detection.

1. In **Arm Control** or **Full Control** tab
2. Ensure **Start Arm** is running
3. Position arm at each scanning viewpoint:
   - **Select**: `front` → **Send Position** (forward view)
   - **Select**: `left` → **Send Position** (left view)
   - **Select**: `right` → **Send Position** (right view)
   - **Select**: `up` → **Send Position** (overhead view)
   - **Select**: `down` → **Send Position** (ground view)
4. OAK-D camera publishes detections to `/detected_objects` topic
5. View detections (not yet in UI):
   - Terminal: `ros2 topic echo /detected_objects`
   - Or use RViz with detection visualization plugin

**Future Enhancement:** UI may integrate ObjectID visual feedback.

### Workflow 5: Direct Joint Control (Teaching)

**[SCREENSHOT PLACEHOLDER: Workflow diagram for joint control]**

**Use Case:** Manually teaching robot positions for later playback.

1. Ensure either **Arm Control "Start Arm"** or **Full Control** is running
2. Go to **Joint Control** tab
3. Click **Read Current Joint Positions**
   - All sliders and input fields now enabled
   - Current positions displayed
4. Adjust joint positions:
   - Method A: Drag slider
   - Method B: Type value in input field
   - **Important**: Only change ONE joint at a time
5. Set **Time from start**: e.g., 2.0 seconds for smooth motion
6. Click **Publish Joint Trajectory**
   - Safety check: Verifies only 1 joint changed
   - Trajectory executed
7. Repeat steps 3-6 for next joint or position
8. Record final joint values for predefined positions (add to `initial_positions.yaml`)

**Safety Enforcement:**
```
If multiple joints changed:
✗ ERROR: Multiple joints changed! Only 1 joint can move at a time.
  Shoulder Pan: 0.021 rad
  Elbow: 0.012 rad
Trajectory blocked for safety.
```

### Workflow 6: Emergency Stop Procedure

**[SCREENSHOT PLACEHOLDER: Emergency stop workflow diagram]**

**Scenario:** Robot exhibiting unsafe behavior or unexpected motion.

**Immediate Action:**
1. **Click Emergency Stop button** (large red button)
   - Robot immediately stops all motion
   - Button turns dark red: "EMERGENCY STOP ACTIVE"
   - Status: "⚠ EMERGENCY STOP - Published stop signal"
2. Assess situation:
   - Check for collisions or obstacles
   - Verify workspace is clear
   - Identify root cause

**Recovery:**
3. Resolve issue (remove obstacle, fix configuration, etc.)
4. **Click Emergency Stop button again** to release
   - Robot safety system resets
   - Status: "✓ EMERGENCY STOP RELEASED"
   - Button returns to normal red: "EMERGENCY STOP (Click to Activate)"
5. Robot returns to operational mode
6. Resume operations

**Notes:**
- Emergency stop button synchronized across Arm Control and Full Control tabs
- Cancels any active trajectory execution
- Requires manual release (not automatic timeout)

---

## Troubleshooting

Common issues and solutions.

### Issue: Tab Grayed Out / Disabled

**Symptom:** Can't click on a tab, it appears disabled/grayed out.

**Cause:** Another operational mode is active.

**Solution:**
1. Go to the active tab (the one that's enabled)
2. Stop all running processes:
   - Click "Stop Arm"
   - Click "Stop Mapping"
   - Click "Stop Localization"
   - etc.
3. Wait for all processes to terminate (buttons return to "Start" state)
4. Now other tabs should become enabled

### Issue: "Start Arm" Fails Immediately

**Symptom:** Click "Start Arm", status shows error, button returns to "Start" state.

**Possible Causes & Solutions:**

1. **Robot not reachable (real hardware):**
   - Check network connection: `ping 192.168.1.102`
   - Verify robot is powered on
   - Check robot is in remote control mode (teach pendant)

2. **URSim not running:**
   - Ensure Docker is installed: `docker --version`
   - Check URSim container: `docker ps | grep ursim`
   - Start URSim manually if needed

3. **Incorrect configuration:**
   - Simulation=false but robot not available → Set Simulation=true
   - URSim=true but no Docker → Install Docker or set URSim=false

4. **Port conflict:**
   - Another program connected to robot
   - Close other control programs (UR Polyscope, etc.)

### Issue: Emergency Stop Won't Release

**Symptom:** Click emergency stop release, but robot doesn't respond.

**Solution:**
1. Check status text for error messages
2. If using real robot:
   - Check teach pendant for safety popup
   - Manually clear protective stop on teach pendant
   - Press "Unlock protective stop" and "Close safety popup" on pendant
3. Robot dashboard connection may have failed:
   - Verify robot IP in status text
   - Check network connection
   - Restart arm control: Stop Arm → Start Arm

4. If using URSim:
   - URSim may have frozen - restart Docker container
   - Check Docker logs: `docker logs <ursim_container_id>`

### Issue: Joint Control "Publish" Does Nothing

**Symptom:** Click "Publish Joint Trajectory", no motion occurs.

**Possible Causes:**

1. **Haven't read positions first:**
   - Solution: Click "Read Current Joint Positions" first

2. **Multiple joints changed:**
   - Status shows: "Multiple joints changed!" error
   - Solution: Reset one joint to original value, only change 1 joint

3. **Planner not running:**
   - Solution: Ensure "Start Arm" is running in Arm Control or Full Control tab

4. **Topic mismatch:**
   - Check namespace configuration (especially in Full Control with hybrid sim)
   - Verify with: `ros2 topic list | grep planned_trajectory`

### Issue: Distance Sensors Return All Zeros

**Symptom:** Status shows distance readings as [0, 0, 0].

**Possible Causes:**

1. **Sensors not connected (real hardware):**
   - Check Arduino USB connection
   - Run: `ls /dev/ttyUSB* /dev/ttyACM*` to find device
   - Check sensor wiring and power

2. **Wrong simulation mode:**
   - Using `arduino_sensors` instead of `arduino_sensors_sim`
   - Solution: UI should auto-select correct version based on Simulation setting

3. **Sensor initialization failed:**
   - Stop distance sensors
   - Check status text for initialization errors
   - Restart distance sensors

### Issue: ROS2 Processes Keep Running After Closing UI

**Symptom:** `ros2 node list` shows nodes still running after closing UI.

**Cause:** UI cleanup didn't complete properly.

**Solution:**
1. Identify zombie processes:
   - Run in terminal: `ps aux | grep ros2`
2. Kill manually:
   - `killall -9 ros2`
   - Or: `kill -9 <PID>` for specific processes
3. Clean Gazebo:
   - `pkill -9 gz`
   - `killall gzserver gzclient`

**Prevention:**
- Always stop all processes before closing UI
- Use UI's "List ROS2 Processes" and "Kill Process" features

### Issue: Topics Not Appearing in Full Control "Refresh"

**Symptom:** Topic dropdown empty or missing known topics.

**Possible Causes:**

1. **ROS node not running:**
   - Topics only exist when publishing node is active
   - Solution: Launch required nodes first

2. **ROS_DOMAIN_ID mismatch:**
   - Check: `echo $ROS_DOMAIN_ID`
   - Ensure UI launched with same domain as nodes

3. **Command failed:**
   - Check status text for errors
   - Try manually: `ros2 topic list`

### Issue: "View Map" Shows Empty Database

**Symptom:** RTABMap viewer opens but shows no map or very few points.

**Possible Causes:**

1. **Mapping never run:**
   - Solution: Create a map first with "Start Mapping"

2. **Wrong database file:**
   - UI looks for: `~/ros2_ws/src/navi_wall/maps/rtabmap.db`
   - Check file exists: `ls ~/ros2_ws/src/navi_wall/maps/`

3. **Mapping stopped prematurely:**
   - Database save interrupted
   - Solution: Create new map, ensure "Stop Mapping" completes cleanly

### Issue: MoveIt Planning Fails

**Symptom:** Using Planner Backend=moveit, but trajectories fail or hang.

**Note:** MoveIt integration is **experimental and under testing**.

**Troubleshooting:**
1. Check MoveIt logs in status text
2. Verify planning scene loaded correctly
3. Try increasing planning time
4. If persistent issues: Switch to Planner Backend=legacy

**Recommendation:** Use legacy planner for production work.

---

## Safety and Best Practices

### General Safety

1. **Emergency Stop Accessibility:**
   - Always know where emergency stop button is
   - Keep finger near button when testing new trajectories
   - Physical e-stop button on robot should be accessible

2. **Workspace Clearance:**
   - Keep workspace clear of personnel during operation
   - Use barriers or safety mats for human detection
   - Never enter workspace while robot is moving

3. **Simulation First:**
   - Test all new trajectories in simulation before real hardware
   - Use URSim for realistic kinematic validation
   - Verify collision avoidance works as expected

4. **Speed Scaling:**
   - Start with reduced speed (50% or lower)
   - Increase gradually after validation
   - UR robot has built-in speed scaling for safety

### UI-Specific Best Practices

1. **Process Management:**
   - Stop all processes before closing UI
   - Wait for clean shutdown (SIGINT → SIGTERM → SIGKILL sequence)
   - Use "List ROS2 Processes" to verify cleanup

2. **Configuration Validation:**
   - Double-check configuration before clicking "Start"
   - Simulation vs. Real hardware setting is critical
   - IP address automatically configured, but verify in status text

3. **Status Monitoring:**
   - Always monitor status text for errors
   - Don't ignore warnings (yellow messages)
   - Use search to find specific errors in long logs

4. **Tab Discipline:**
   - Only use one operational mode at a time
   - Let UI enforce tab mutual exclusivity
   - Don't force-kill processes if tab is disabled (stop from correct tab first)

5. **Joint Control Safety:**
   - Always use "Read Current Joint Positions" first
   - Never override single-joint safety constraint
   - Use longer time-from-start for safer, smoother motion

### Robot Dashboard Best Practices

1. **Status Before Control:**
   - Read robot status before sending control commands
   - Use "Send All Status Commands" to get full picture
   - Verify robot is in expected state

2. **Safe Power Sequence:**
   - Power on → Wait for initialization → Brake release → Load program → Play
   - Don't skip steps

3. **Protective Stop Recovery:**
   - Don't immediately restart after protective stop
   - Investigate cause first
   - Use dashboard commands in correct order: Close popup → Unlock stop → Play

### Planning Best Practices

1. **Legacy Planner:**
   - Verify reachability maps loaded correctly
   - Use "Reset Planner" if behavior seems incorrect
   - Planner state can accumulate constraints over time

2. **MoveIt (Experimental):**
   - Expect issues - this is under testing
   - Fall back to legacy if critical operations needed
   - Report issues for improvement

3. **Trajectory Validation:**
   - Check planned path in RViz before execution
   - Verify no self-collisions or environment collisions
   - Monitor execution, ready to emergency stop

### Data Management

1. **Map Files:**
   - Back up maps regularly
   - Name maps descriptively (not just "rtabmap.db")
   - Store maps outside of workspace for safety

2. **Logs:**
   - Save important status text results (copy/paste)
   - Use Clear/Restore to manage long logs
   - Screenshot errors for troubleshooting later

---

## Advanced Features

### Terminal Widgets (QTermWidget)

Some tabs include terminal emulators for direct shell interaction.

**[SCREENSHOT PLACEHOLDER: Terminal widget in Joint Control]**

**Features:**
- Full bash shell with environment variables
- Execute ROS2 commands directly
- Color support (ANSI codes)
- Scrollback buffer

**Usage:**
- Type commands directly in terminal
- Output shown in terminal (not status text)
- Useful for experimentation and debugging

**Tabs with Terminals:**
- Base Control (left side)
- Joint Control (left side)

### Process Kill Confirmation

**[SCREENSHOT PLACEHOLDER: Kill process confirmation dialog]**

When killing a process via "Kill Process" button:
1. Confirmation dialog appears: "Are you sure you want to kill process XXXX?"
2. Shows PID and command name
3. Options: "Yes" or "No"
4. "Yes" → `kill -9 <PID>` executed
5. Process list auto-refreshes

**Warning:** This sends SIGKILL (immediate termination), not graceful shutdown.

### Emergency Stop Synchronization

Emergency stop buttons in Arm Control and Full Control tabs are **synchronized**:
- Clicking one updates visual state of both
- Both publish to same topic (context-dependent)
- External emergency stop publishers also update UI state

**[SCREENSHOT PLACEHOLDER: Both emergency stop buttons shown side-by-side]**

**Implementation:**
- Internal state variable: `self.emergency_stop_active`
- Topic subscriber: `/emergency_stop` (with QoS: RELIABLE + TRANSIENT_LOCAL)
- Prevents feedback loops: UI-triggered changes don't re-trigger from subscription

---

## Tips and Tricks

1. **Quick Status Search:**
   - Use search to find specific errors: Type "error", click ▶ to jump between errors
   - Search for "✓" to see all success messages
   - Search for "⚠" to find warnings

2. **Process Monitoring:**
   - "List ROS2 Processes" shows ALL ros2 processes, not just UI-launched ones
   - Use this to find orphaned nodes from previous sessions
   - Hover over process in dropdown to see full command line

3. **Configuration Presets:**
   - Common configurations:
     - **Testing**: Simulation=true, Planner=legacy, Headless=false
     - **Production**: Simulation=false, Planner=legacy
     - **Development**: Simulation=true, URSim=true, Planner=legacy
   - Remember your settings - they persist within the session

4. **Terminal Commands While UI Active:**
   - You can run ROS2 commands in external terminal while UI is running
   - Useful for monitoring: `ros2 topic echo`, `ros2 topic hz`, etc.
   - Avoid launching conflicting nodes (UI won't know about them)

5. **Graceful Shutdown:**
   - Always stop processes before closing UI
   - If UI becomes unresponsive: Close window, then clean up processes manually
   - Use `killall -9 ros2` as last resort only

6. **Debugging Launch Issues:**
   - If launch fails, status text shows full error
   - Copy entire command from status (starts with ▶)
   - Run manually in terminal to see more detailed error output

7. **Multiple Map Management:**
   - UI uses default name "rtabmap.db"
   - After mapping, rename file: `mv rtabmap.db warehouse_map_2024-03-27.db`
   - When using localization, create symlink: `ln -s warehouse_map_2024-03-27.db rtabmap.db`

---

## Frequently Asked Questions

**Q: Can I run multiple UI instances simultaneously?**

A: Not recommended. Multiple UIs may conflict when launching processes or controlling the robot. If you need multiple views, use one UI and external terminals for monitoring.

**Q: Why is Joint Control tab sometimes disabled?**

A: Joint Control is disabled when Planner Backend=moveit. MoveIt provides its own motion planning interface, making direct joint trajectory publishing incompatible.

**Q: What's the difference between "Stop" and "Kill Process"?**

A: "Stop" (green button turning back to normal) uses graceful shutdown (SIGINT → SIGTERM → SIGKILL with delays). "Kill Process" immediately sends SIGKILL with no grace period.

**Q: Can I add my own predefined positions to the dropdown?**

A: Yes, edit `config/initial_positions.yaml` in the arm_control package. Add your position with joint values, rebuild package, restart UI.

**Q: Why does simulation mode have two options (URSim and regular)?**

A: Regular simulation uses Gazebo (fast, no robot dashboard). URSim uses Universal Robots' official simulator in Docker (realistic, with dashboard interface, but requires Docker and more resources).

**Q: The UI shows ANSI color codes as text instead of colors. How to fix?**

A: This shouldn't happen - UI converts ANSI to HTML automatically. If you see raw codes, there may be a PyQt5 issue. Try upgrading: `pip3 install --upgrade PyQt5`.

**Q: Can I customize the UI layout or colors?**

A: UI is implemented in Python (PyQt5), so you can edit `control/UI.py` directly. Be careful with button signal connections and process management logic. Consider forking and customizing for your specific needs.

**Q: Why do some buttons say "namespace not available" in simulation?**

A: This can happen if hybrid_sim setting doesn't match the actual running system. Stop all processes, verify configuration, restart with correct settings.

---

## Contributing

If you find bugs or want to suggest improvements to the UI:

1. Document the issue:
   - What you did (step-by-step)
   - What you expected
   - What actually happened
   - Screenshot of UI state and status text
2. Check if configuration was correct for your use case
3. Report to package maintainers with full details

**Common Enhancement Requests:**
- Additional keyboard shortcuts
- Customizable button layouts
- Multi-language support
- Saved configuration profiles
- Integration of hyperspectral camera controls (planned)
- GPR sensor controls (planned)

---

## Appendix: Technical Details

### Architecture

- **Framework**: PyQt5
- **ROS2 Integration**: rclpy with 10Hz spin timer
- **Process Management**: QProcess with signal handlers
- **Terminal Integration**: QTermWidget (libqtermwidget5-0)
- **ANSI-to-HTML**: Custom converter for GitHub dark theme colors

### File Structure

```
arm_control/
├── control/
│   ├── UI.py                         # Main UI implementation
│   └── __pycache__/
└── UI_utils/
    └── qtermwidget_wrapper.py        # QTermWidget wrapper
```

### Dependencies

**Python Packages:**
- `PyQt5` - GUI framework
- `rclpy` - ROS2 Python client library
- `ament_index_python` - ROS2 package resource lookup

**System Libraries:**
- `libqtermwidget5-0` - Terminal emulator widget
- Qt5 libraries (installed with PyQt5)

**ROS2 Message Types:**
- `geometry_msgs/Pose` - Goal poses
- `std_msgs/Bool` - Emergency stop
- `sensor_msgs/JointState` - Joint positions
- `trajectory_msgs/JointTrajectory` - Joint trajectories
- `control_msgs/FollowJointTrajectory` (action) - Trajectory execution
- `ur_msgs/IOStates` - UR I/O states
- `ur_msgs/SetIO` (service) - UR I/O control

### QoS Profiles

Emergency stop uses specific QoS for reliability:
```python
QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL
)
```

This ensures:
- Late-joining subscribers see last emergency stop state
- Reliable delivery (no message loss)
- Only latest state kept (depth=1)

---

## Version History

- **v1.0** (Current) - Initial UI release with 4-tab layout, process management, emergency stop, and full integration

---

**For more information on the arm_control package capabilities, see [README.md](README.md)**

**Package Maintainer**: [Contact information]  
**Last Updated**: March 2026
