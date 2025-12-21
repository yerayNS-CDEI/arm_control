cmake_minimum_required(VERSION 3.5)
project(arm_control)

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
find_package(rosidl_default_runtime REQUIRED)

# Generate service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ComputeBasePlacement.srv"
  "srv/ComputeWallDiscretization.srv"
  "srv/OptimalBase.srv"
  "srv/HyperspectralCommand.srv"
  "srv/HyperspectralConfig.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

# Find Python and install Python package manually
find_package(Python3 REQUIRED COMPONENTS Interpreter)

# Install Python modules to local site-packages
install(
  DIRECTORY arm_control/
  DESTINATION "local/lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/dist-packages/${PROJECT_NAME}"
  FILES_MATCHING
  PATTERN "*.py"
  PATTERN "__pycache__" EXCLUDE
)

# Install Python executables (rename to remove .py extension)
install(PROGRAMS arm_control/control/end_effector_pose_node.py DESTINATION lib/${PROJECT_NAME} RENAME end_effector_pose_node)
install(PROGRAMS arm_control/control/example_move.py DESTINATION lib/${PROJECT_NAME} RENAME example_move)
install(PROGRAMS arm_control/control/publisher_joint_trajectory_controller.py DESTINATION lib/${PROJECT_NAME} RENAME publisher_joint_trajectory_controller)
install(PROGRAMS arm_control/control/publisher_joint_trajectory_planned.py DESTINATION lib/${PROJECT_NAME} RENAME publisher_joint_trajectory_planned)
install(PROGRAMS arm_control/control/publisher_joint_trajectory_planned_topic.py DESTINATION lib/${PROJECT_NAME} RENAME publisher_joint_trajectory_planned_topic)
install(PROGRAMS arm_control/control/position_sender_node.py DESTINATION lib/${PROJECT_NAME} RENAME position_sender_node)
install(PROGRAMS arm_control/control/exhaustive_scan_node.py DESTINATION lib/${PROJECT_NAME} RENAME exhaustive_scan_node)
install(PROGRAMS arm_control/control/remote_DASHBOARD.py DESTINATION lib/${PROJECT_NAME} RENAME remote_DASHBOARD)
install(PROGRAMS arm_control/control/remote_SCRIPT.py DESTINATION lib/${PROJECT_NAME} RENAME remote_SCRIPT)
install(PROGRAMS arm_control/control/send_and_monitor_trajectory.py DESTINATION lib/${PROJECT_NAME} RENAME send_and_monitor_trajectory)
install(PROGRAMS arm_control/control/UI.py DESTINATION lib/${PROJECT_NAME} RENAME UI)
install(PROGRAMS arm_control/sensors/sensors_distance.py DESTINATION lib/${PROJECT_NAME} RENAME sensors_distance)
install(PROGRAMS arm_control/sensors/sensors_distance_orientation.py DESTINATION lib/${PROJECT_NAME} RENAME sensors_distance_orientation)
install(PROGRAMS arm_control/sensors/sensors_distance_orientation_sim.py DESTINATION lib/${PROJECT_NAME} RENAME sensors_distance_orientation_sim)
install(PROGRAMS arm_control/sensors/sensors_orientation.py DESTINATION lib/${PROJECT_NAME} RENAME sensors_orientation)
install(PROGRAMS arm_control/planner/base_placement_node.py DESTINATION lib/${PROJECT_NAME} RENAME base_placement_node)
install(PROGRAMS arm_control/planner/planner_node.py DESTINATION lib/${PROJECT_NAME} RENAME planner_node)
install(PROGRAMS arm_control/planner/wall_discretization_node.py DESTINATION lib/${PROJECT_NAME} RENAME wall_discretization_node)
install(PROGRAMS arm_control/planner/visualize_wall_discretization_client.py DESTINATION lib/${PROJECT_NAME} RENAME visualize_wall_discretization_client)
install(PROGRAMS arm_control/sensors/hyperspectral_node.py DESTINATION lib/${PROJECT_NAME} RENAME hyperspectral_node)
install(PROGRAMS arm_control/sensors/test_hyperspectral_client.py DESTINATION lib/${PROJECT_NAME} RENAME test_hyperspectral_client)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install URDF files
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# Install RViz config files
install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

# Install resource files
install(DIRECTORY resource/
  DESTINATION share/${PROJECT_NAME}/resource
)

# Install test files
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
