#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "collision/path_collision_checking.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  const rclcpp::NodeOptions options;
  std::shared_ptr<constrained_manipulability::PathCollisionChecking> node;
  try {
    node = std::make_shared<constrained_manipulability::PathCollisionChecking>(options);
  } catch (const std::exception & ex) {
    // Startup failures (no robot_description, bad URDF, no kinematic chain) abort here rather
    // than leaving a live node that never advertises /collision/check_collision_pose — the arm
    // planner then rejects every goal it cannot collision-validate, with no clue why.
    RCLCPP_FATAL(
      rclcpp::get_logger("path_collision_checking"),
      "Collision checking node failed to start: %s", ex.what());
    rclcpp::shutdown();
    return 1;
  }

  // Use a MultiThreadedExecutor to handle multiple callback groups
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  
  rclcpp::shutdown();

  return 0;
}