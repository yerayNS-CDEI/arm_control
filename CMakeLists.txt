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

# Generate service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ComputeBasePlacement.srv"
  "srv/ComputeWallDiscretization.srv"
  "srv/OptimalBase.srv"
  "srv/HyperspectralCommand.srv"
  "srv/HyperspectralConfig.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

# Install Python modules and scripts via setup.py
find_package(Python3 REQUIRED COMPONENTS Interpreter)
install(CODE "execute_process(
  COMMAND ${Python3_EXECUTABLE} setup.py install --prefix=${CMAKE_INSTALL_PREFIX} --install-scripts=${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)")

# Install test files
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
