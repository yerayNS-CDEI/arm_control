cmake_minimum_required(VERSION 3.5)
project(arm_control)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(backward_ros REQUIRED)

# Generate service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ComputeBasePlacement.srv"
  "srv/ComputeWallDiscretization.srv"
  "srv/OptimalBase.srv"
  "srv/HyperspectralCommand.srv"
  "srv/HyperspectralConfig.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

# Install Python modules as symlinks for development
find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(PYTHON_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/local/lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/dist-packages")

# Create symlinks for Python modules during install
install(CODE "
  execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink
    ${CMAKE_CURRENT_SOURCE_DIR}/control
    ${PYTHON_INSTALL_DIR}/control)
  execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink
    ${CMAKE_CURRENT_SOURCE_DIR}/planner
    ${PYTHON_INSTALL_DIR}/planner)
  execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink
    ${CMAKE_CURRENT_SOURCE_DIR}/sensors
    ${PYTHON_INSTALL_DIR}/sensors)
  execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink
    ${CMAKE_CURRENT_SOURCE_DIR}/UI_utils
    ${PYTHON_INSTALL_DIR}/UI_utils)
")

# Install Python executables as symlinks (for instant updates during development)
# Each executable is symlinked from source to lib/${PROJECT_NAME} without .py extension
install(CODE "
  set(LIB_DIR \"${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}\")
  file(MAKE_DIRECTORY \${LIB_DIR})
  
  # Control nodes
  execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/control/end_effector_pose_node.py \${LIB_DIR}/end_effector_pose_node)
  execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/control/example_move.py \${LIB_DIR}/example_move)
  execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/control/exhaustive_scan_node.py \${LIB_DIR}/exhaustive_scan_node)
  execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/control/freedrive_control.py \${LIB_DIR}/freedrive_control)
  execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/control/position_sender_node.py \${LIB_DIR}/position_sender_node)
  execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/control/publisher_joint_trajectory_controller.py \${LIB_DIR}/publisher_joint_trajectory_controller)
  execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/control/publisher_joint_trajectory_planned.py \${LIB_DIR}/publisher_joint_trajectory_planned)
  execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/control/publisher_joint_trajectory_planned_topic.py \${LIB_DIR}/publisher_joint_trajectory_planned_topic)
  execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/control/remote_DASHBOARD.py \${LIB_DIR}/remote_DASHBOARD)
  execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/control/remote_SCRIPT.py \${LIB_DIR}/remote_SCRIPT)
  execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/control/robot_command_logger.py \${LIB_DIR}/robot_command_logger)
  execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/control/robot_status_check.py \${LIB_DIR}/robot_status_check)
  execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/control/send_and_monitor_trajectory.py \${LIB_DIR}/send_and_monitor_trajectory)
  execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/control/UI.py \${LIB_DIR}/UI)
  
  # Planner nodes
  execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/planner/base_placement_node.py \${LIB_DIR}/base_placement_node)
  execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/planner/planner_node.py \${LIB_DIR}/planner_node)
  execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/planner/visualize_wall_discretization_client.py \${LIB_DIR}/visualize_wall_discretization_client)
  execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/planner/wall_discretization_node.py \${LIB_DIR}/wall_discretization_node)
  
  # Sensor nodes
  execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/sensors/hyperspectral_node.py \${LIB_DIR}/hyperspectral_node)
  execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/sensors/sensors_distance.py \${LIB_DIR}/sensors_distance)
  execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/sensors/sensors_distance_orientation.py \${LIB_DIR}/sensors_distance_orientation)
  execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/sensors/sensors_distance_orientation_sim.py \${LIB_DIR}/sensors_distance_orientation_sim)
  execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/sensors/sensors_orientation.py \${LIB_DIR}/sensors_orientation)
  execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/sensors/arduino_sensors.py \${LIB_DIR}/arduino_sensors)
  execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/sensors/test_hyperspectral_client.py \${LIB_DIR}/test_hyperspectral_client)
")

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.py"
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  FILES_MATCHING PATTERN "*.yaml"
)

# Install URDF files
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
  FILES_MATCHING 
  PATTERN "*.urdf"
  PATTERN "*.xacro"
)

# Install RViz config files
install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
  FILES_MATCHING PATTERN "*.rviz"
)

# Install resource files
install(DIRECTORY resource/
  DESTINATION share/${PROJECT_NAME}/resource
)

# Install UI_utils
install(DIRECTORY UI_utils/
  DESTINATION share/${PROJECT_NAME}/UI_utils
  FILES_MATCHING PATTERN "*.py"
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)

ament_package()
