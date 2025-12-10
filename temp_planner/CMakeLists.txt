cmake_minimum_required(VERSION 3.5)
project(robotic_arm_planner)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

ament_package()
