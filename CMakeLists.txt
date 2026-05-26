cmake_minimum_required(VERSION 3.5)
project(arm_control)

# C++17 required by moveit_core headers
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_eigen_kdl REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(fcl REQUIRED)
find_package(assimp REQUIRED)
find_package(resource_retriever REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_library(EIGEN_CDDLIB eigen-cddlib)
find_package(octomap_filter_interfaces REQUIRED)
find_package(robot_collision_checking REQUIRED)
find_package(backward_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)

# Generate service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Matrix.msg"
  "msg/ObjectDistances.msg"
  "msg/Polytope.msg"
  "srv/AddRemoveCollisionMesh.srv"
  "srv/AddRemoveCollisionSolid.srv"
  "srv/AddRemoveCollisionMeshStamped.srv"
  "srv/AddRemoveCollisionSolidStamped.srv"
  "srv/GetJacobianMatrix.srv"
  "srv/GetPolytopes.srv"
  "srv/GetSlicedPolytope.srv"
  "srv/UpdateCollisionPose.srv"
  "srv/UpdateCollisionPoseStamped.srv"
  "srv/CheckCollisionPose.srv"
  "srv/CheckCollisionPoses.srv"
  "srv/ComputeBasePlacement.srv"
  "srv/ComputeWallDiscretization.srv"
  "srv/OptimalBase.srv"
  "srv/HyperspectralCommand.srv"
  "srv/HyperspectralConfig.srv"
  "srv/SendPosition.srv"
  "srv/ScriptCommand.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs sensor_msgs shape_msgs std_msgs
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
  execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/control/joint_state_merger_node.py \${LIB_DIR}/joint_state_merger_node)
  execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/control/position_sender_node.py \${LIB_DIR}/position_sender_node)
  execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/control/script_command_client_example.py \${LIB_DIR}/script_command_client_example)
  execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/control/script_command_service_node.py \${LIB_DIR}/script_command_service_node)
  execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/control/publisher_joint_trajectory_planned.py \${LIB_DIR}/publisher_joint_trajectory_planned)
  execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/control/remote_DASHBOARD.py \${LIB_DIR}/remote_DASHBOARD)
  execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/control/remote_SCRIPT.py \${LIB_DIR}/remote_SCRIPT)
  execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/control/robot_command_logger.py \${LIB_DIR}/robot_command_logger)
  execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/control/robot_status_check.py \${LIB_DIR}/robot_status_check)
  execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/control/send_and_monitor_trajectory.py \${LIB_DIR}/send_and_monitor_trajectory)
  execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/control/UI.py \${LIB_DIR}/UI)
  
  # Planner nodes
  execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/planner/base_placement_node.py \${LIB_DIR}/base_placement_node)
  execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/planner/planner_node.py \${LIB_DIR}/planner_node)
  execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/planner/moveit_planner_node.py \${LIB_DIR}/moveit_planner_node)
  execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/planner/visualize_wall_discretization_client.py \${LIB_DIR}/visualize_wall_discretization_client)
  execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/planner/wall_discretization_node.py \${LIB_DIR}/wall_discretization_node)
  execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/planner/optimal_base_service.py \${LIB_DIR}/optimal_base_service)
  
  # Sensor nodes
  execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/sensors/hyperspectral_node.py \${LIB_DIR}/hyperspectral_node)
  execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/sensors/sensors_distance.py \${LIB_DIR}/sensors_distance)
  execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/sensors/sensors_distance_orientation.py \${LIB_DIR}/sensors_distance_orientation)
  execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/sensors/sensors_distance_orientation_sim.py \${LIB_DIR}/sensors_distance_orientation_sim)
  execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/sensors/align_ee_to_wall.py \${LIB_DIR}/align_ee_to_wall)
  execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/sensors/arduino_sensors.py \${LIB_DIR}/arduino_sensors)
  execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/sensors/arduino_sensors_sim.py \${LIB_DIR}/arduino_sensors_sim)
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

# Install SRDF files
install(DIRECTORY srdf/
  DESTINATION share/${PROJECT_NAME}/srdf
  FILES_MATCHING
  PATTERN "*.srdf"
  PATTERN "*.xacro"
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

set(collision_cpp_dependencies
  rclcpp
  geometry_msgs
  std_msgs
  sensor_msgs
  shape_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_eigen
  tf2_eigen_kdl
  tf2_kdl
  kdl_parser
  urdf
  geometric_shapes
  octomap_msgs
  pcl_ros
  fcl
  assimp
  resource_retriever
  octomap_filter_interfaces
  robot_collision_checking
)

add_library(collision_core SHARED
  collision/src/constrained_manipulability_mod.cpp
  collision/src/path_collision_checking.cpp
  collision/src/constrained_manipulability_utils.cpp
  collision/src/polytope.cpp
)
target_include_directories(collision_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/collision/include>
  $<INSTALL_INTERFACE:include>
  ${Eigen3_INCLUDE_DIRS}
)
target_link_libraries(collision_core
  Boost::boost
  ${EIGEN_CDDLIB}
  ${OCTOMAP_LIBRARIES}
  ${fcl_LIBRARIES}
  ${assimp_LIBRARIES}
)
ament_target_dependencies(collision_core ${collision_cpp_dependencies})
rosidl_get_typesupport_target(arm_control_interfaces_typesupport ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(collision_core ${arm_control_interfaces_typesupport})

add_executable(constrained_manipulability_node_mod
  collision/src/constrained_manipulability_mod_main.cpp
)
target_link_libraries(constrained_manipulability_node_mod collision_core)
ament_target_dependencies(constrained_manipulability_node_mod ${collision_cpp_dependencies})

add_executable(path_collision_checking
  collision/src/path_collision_checking_main.cpp
)
target_link_libraries(path_collision_checking collision_core)
ament_target_dependencies(path_collision_checking ${collision_cpp_dependencies})

install(TARGETS
  collision_core
  constrained_manipulability_node_mod
  path_collision_checking
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY collision/include/
  DESTINATION include
)

install(DIRECTORY collision/
  DESTINATION share/${PROJECT_NAME}/collision
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.cpp"
# C++ EE Jacobian node
add_executable(ee_jacobian_node control/ee_jacobian_node.cpp)
ament_target_dependencies(ee_jacobian_node
  rclcpp
  sensor_msgs
  nav_msgs
  std_msgs
  tf2_ros
  moveit_core
  moveit_ros_planning
)
install(TARGETS ee_jacobian_node
  DESTINATION lib/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(${collision_cpp_dependencies})

ament_package()
