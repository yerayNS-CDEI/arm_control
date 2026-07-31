// Publishes end-effector pose and Cartesian velocity using MoveIt's analytic Jacobian.
//
// Pose source  : TF2  (arm_base → arm_tool0)  — identical to end_effector_pose_node.py
// Jacobian     : MoveIt RobotState::getJacobian()  — analytic, arm joints only
// Velocity     : v_ee = J_arm_base * q_dot
//
// Topics published
//   ~/end_effector_info  (nav_msgs/Odometry)
//       pose  – EE pose in arm_base from TF
//       twist – Cartesian velocity v = J(q) * q_dot in arm_base frame
//
//   ~/jacobian  (std_msgs/Float64MultiArray)
//       6×6 geometric Jacobian in arm_base frame, row-major

#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

class EEJacobianNode : public rclcpp::Node
{
public:
  explicit EEJacobianNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("ee_jacobian_node", options)
  {
    declare_parameter("group_name",   "arm_manipulator");
    declare_parameter("ee_link",      "arm_tool0");
    declare_parameter("base_frame",   "arm_base");
    declare_parameter("publish_rate", 50.0);

    publish_rate_ = get_parameter("publish_rate").as_double();

    // TF2 – used for the EE pose and for rotating the Jacobian into arm_base
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) { jointStateCb(msg); });

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("~/end_effector_info", 10);
    jac_pub_  = create_publisher<std_msgs::msg::Float64MultiArray>("~/jacobian", 10);

    // shared_from_this() is valid only after construction → one-shot timer
    init_timer_ = create_wall_timer(
      std::chrono::milliseconds(0),
      [this]() { init_timer_->cancel(); initRobotModel(); });
  }

private:
  // -----------------------------------------------------------------------
  // Initialization (runs once after construction)
  // -----------------------------------------------------------------------

  void initRobotModel()
  {
    const std::string group = get_parameter("group_name").as_string();
    const std::string ee    = get_parameter("ee_link").as_string();

    loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(
      shared_from_this(), "robot_description");
    model_ = loader_->getModel();

    if (!model_) {
      RCLCPP_ERROR(get_logger(), "Failed to load robot model from 'robot_description'");
      return;
    }

    RCLCPP_INFO(get_logger(), "Robot model: '%s'  frame: '%s'",
      model_->getName().c_str(), model_->getModelFrame().c_str());

    jmg_ = model_->getJointModelGroup(group);
    if (!jmg_) {
      RCLCPP_ERROR(get_logger(), "Joint model group '%s' not found", group.c_str());
      return;
    }

    ee_link_ = model_->getLinkModel(ee);
    if (!ee_link_) {
      RCLCPP_ERROR(get_logger(), "Link '%s' not found in robot model", ee.c_str());
      return;
    }

    {
      std::lock_guard<std::mutex> lk(mtx_);
      state_ = std::make_shared<moveit::core::RobotState>(model_);
      state_->setToDefaultValues();
      state_->update();
    }

    publish_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_),
      [this]() { publishCb(); });

    RCLCPP_INFO(get_logger(),
      "EE Jacobian node ready — group: %s  ee_link: %s  rate: %.0f Hz",
      group.c_str(), ee.c_str(), publish_rate_);
  }

  // -----------------------------------------------------------------------
  // Joint-state callback — updates MoveIt robot state for getJacobian()
  // -----------------------------------------------------------------------

  void jointStateCb(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!state_) return;

    for (size_t i = 0; i < msg->name.size(); ++i) {
      const moveit::core::JointModel * jm = model_->getJointModel(msg->name[i]);
      if (jm && i < msg->position.size()) {
        state_->setJointPositions(jm, &msg->position[i]);
      }
    }
    state_->update();
    js_ = msg;
  }

  // -----------------------------------------------------------------------
  // Helper: convert TF quaternion → Eigen rotation matrix
  // -----------------------------------------------------------------------

  static Eigen::Matrix3d rotFromTF(const geometry_msgs::msg::Quaternion & q)
  {
    return Eigen::Quaterniond(q.w, q.x, q.y, q.z).toRotationMatrix();
  }

  // -----------------------------------------------------------------------
  // Publish callback
  // -----------------------------------------------------------------------

  void publishCb()
  {
    const std::string base_frame = get_parameter("base_frame").as_string();
    const std::string ee_name    = get_parameter("ee_link").as_string();

    // --- 1. EE pose from TF (matches end_effector_pose_node.py exactly) ---
    geometry_msgs::msg::TransformStamped t_ee;
    try {
      t_ee = tf_buffer_->lookupTransform(base_frame, ee_name, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "TF lookup %s→%s failed: %s", base_frame.c_str(), ee_name.c_str(), ex.what());
      return;
    }

    // --- 2. Rotation from MoveIt model root → arm_base (for Jacobian rows) -
    // We look this up via TF using the model's own root frame name.
    // Falls back to Identity if the model root is not in the TF tree
    // (in that case the Jacobian is already in a frame ≈ arm_base).
    Eigen::Matrix3d R_root_to_base = Eigen::Matrix3d::Identity();
    const std::string model_frame  = model_ ? model_->getModelFrame() : "";
    if (!model_frame.empty() && model_frame != base_frame) {
      try {
        auto t_base = tf_buffer_->lookupTransform(model_frame, base_frame, tf2::TimePointZero);
        R_root_to_base = rotFromTF(t_base.transform.rotation);
      } catch (const tf2::TransformException &) {
        // silently keep Identity — Jacobian rows stay in model-root frame
      }
    }

    // --- 3. MoveIt robot-state snapshot for getJacobian() ------------------
    moveit::core::RobotState snapshot(model_);
    sensor_msgs::msg::JointState::SharedPtr js;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      if (!state_ || !js_) return;
      snapshot = *state_;
      js       = js_;
    }

    // --- 4. Analytic Jacobian in model-root frame --------------------------
    Eigen::MatrixXd J;
    if (!snapshot.getJacobian(jmg_, ee_link_, Eigen::Vector3d::Zero(), J)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "getJacobian() failed");
      return;
    }

    // --- 5. Rotate Jacobian rows into arm_base frame -----------------------
    // J rows 0-2: linear velocity,  rows 3-5: angular velocity
    // Both are expressed in model root; we need them in arm_base.
    // R_root_to_base * v_base = v_root  →  v_base = R^T * v_root
    J.topRows(3)    = R_root_to_base.transpose() * J.topRows(3);
    J.bottomRows(3) = R_root_to_base.transpose() * J.bottomRows(3);

    // --- 6. Joint velocities mapped to group variable order ----------------
    const auto & var_names = jmg_->getVariableNames();
    const int    n         = static_cast<int>(var_names.size());
    Eigen::VectorXd q_dot  = Eigen::VectorXd::Zero(n);

    if (!js->velocity.empty()) {
      for (int i = 0; i < n; ++i) {
        auto it = std::find(js->name.begin(), js->name.end(), var_names[i]);
        if (it != js->name.end()) {
          const size_t idx = std::distance(js->name.begin(), it);
          if (idx < js->velocity.size()) {
            q_dot[i] = js->velocity[idx];
          }
        }
      }
    }

    const Eigen::VectorXd v_ee = J * q_dot;   // [vx, vy, vz, wx, wy, wz] in arm_base

    // --- 7. Build and publish Odometry -------------------------------------
    const auto now = get_clock()->now();

    nav_msgs::msg::Odometry odom;
    odom.header.stamp    = now;
    odom.header.frame_id = base_frame;
    odom.child_frame_id  = ee_name;

    const auto & tr = t_ee.transform.translation;
    const auto & ro = t_ee.transform.rotation;
    odom.pose.pose.position.x    = tr.x;
    odom.pose.pose.position.y    = tr.y;
    odom.pose.pose.position.z    = tr.z;
    odom.pose.pose.orientation.x = ro.x;
    odom.pose.pose.orientation.y = ro.y;
    odom.pose.pose.orientation.z = ro.z;
    odom.pose.pose.orientation.w = ro.w;

    odom.twist.twist.linear.x  = v_ee[0];
    odom.twist.twist.linear.y  = v_ee[1];
    odom.twist.twist.linear.z  = v_ee[2];
    odom.twist.twist.angular.x = v_ee[3];
    odom.twist.twist.angular.y = v_ee[4];
    odom.twist.twist.angular.z = v_ee[5];

    odom_pub_->publish(odom);

    // --- 8. Jacobian matrix (6 × n, row-major) -----------------------------
    std_msgs::msg::Float64MultiArray jac_msg;
    jac_msg.layout.dim.resize(2);
    jac_msg.layout.dim[0].label  = "rows";
    jac_msg.layout.dim[0].size   = 6;
    jac_msg.layout.dim[0].stride = 6 * n;
    jac_msg.layout.dim[1].label  = "cols";
    jac_msg.layout.dim[1].size   = n;
    jac_msg.layout.dim[1].stride = n;
    jac_msg.data.resize(6 * n);

    for (int r = 0; r < 6; ++r)
      for (int c = 0; c < n; ++c)
        jac_msg.data[r * n + c] = J(r, c);

    jac_pub_->publish(jac_msg);
  }

  // -----------------------------------------------------------------------
  // Members
  // -----------------------------------------------------------------------

  double publish_rate_;

  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  robot_model_loader::RobotModelLoaderPtr   loader_;
  moveit::core::RobotModelPtr               model_;
  moveit::core::RobotStatePtr               state_;
  const moveit::core::JointModelGroup *     jmg_{nullptr};
  const moveit::core::LinkModel *           ee_link_{nullptr};

  std::mutex                                            mtx_;
  sensor_msgs::msg::JointState::SharedPtr               js_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr  joint_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr          odom_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr jac_pub_;
  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EEJacobianNode>());
  rclcpp::shutdown();
  return 0;
}
