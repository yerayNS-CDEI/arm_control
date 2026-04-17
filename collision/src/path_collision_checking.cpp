#include <cassert>
#include <chrono>

#include <rclcpp/logging.hpp>

#include <geometric_shapes/shape_to_marker.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <shape_msgs/msg/mesh.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include "arm_control/msg/matrix.hpp"

#include "collision/path_collision_checking.hpp"

#include <set>
#include <cmath>
#include <Eigen/Dense>
#include <geometry_msgs/msg/pose.hpp>

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace constrained_manipulability
{
PathCollisionChecking::PathCollisionChecking(const rclcpp::NodeOptions& options) : Node("path_collision_checking", options)
{
    // Populate private properties
    base_link_ = this->declare_parameter<std::string>("root", "base_link");
    tip_ = this->declare_parameter<std::string>("tip", "ee_link");

    distance_threshold_ = this->declare_parameter<double>("distance_threshold", 0.3);
    dangerfield_ = this->declare_parameter<double>("dangerfield", 10.0);

    double lin_limit = this->declare_parameter<double>("linearization_limit", 0.1);

    filter_robot_ = this->declare_parameter<bool>("filter_robot", false);

    // TF properties
    buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);

    // Can set robot_description name from parameters
    std::string robot_description_name = "robot_description";
    this->get_parameter_or("robot_description_name", robot_description_name, robot_description_name);

    // Create collision world
    collision_world_ = std::make_shared<robot_collision_checking::FCLInterfaceCollisionWorld>(base_link_);

    // Create parameter client to grab the robot_description from another node (robot_state_publisher)
    // Since this node is in a namespace, we need to use the full path to robot_state_publisher
    std::string robot_state_pub_node = this->get_namespace();
    if (robot_state_pub_node == "/" || robot_state_pub_node.empty()) {
        robot_state_pub_node = "/robot_state_publisher";
    } else {
        robot_state_pub_node += "/robot_state_publisher";
    }
    RCLCPP_INFO(this->get_logger(), "Connecting to robot_state_publisher at: %s", robot_state_pub_node.c_str());
    
    auto params_client = std::make_shared<rclcpp::SyncParametersClient>(this, robot_state_pub_node);
    while (!params_client->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_FATAL(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            rclcpp::shutdown();
        }
        RCLCPP_INFO(this->get_logger(), "Robot state description service not available, waiting...");
    }

    if (!params_client->has_parameter(robot_description_name))
    {
        RCLCPP_FATAL(this->get_logger(), "Parameter %s not found in node robot_state_publisher", robot_description_name.c_str());
        rclcpp::shutdown();
    }

    // Grab URDF robot description
    auto parameters = params_client->get_parameters({ robot_description_name });
    std::string robot_desc_string = parameters[0].value_to_string();
    
    RCLCPP_INFO(this->get_logger(), "Robot description length: %zu bytes", robot_desc_string.length()
);

    // Initialize URDF model
    model_ = std::make_unique<urdf::Model>();

    // Verify that URDF string is in correct format
    if (!model_->initString(robot_desc_string))
    {
        RCLCPP_FATAL(this->get_logger(),"URDF string is not a valid robot model.");
        rclcpp::shutdown();
    }

    // Robot kinematics model creation as KDL tree using URDF model
    if (!kdl_parser::treeFromUrdfModel(*model_, tree_))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to construct KDL tree");
    }
    
    // Print all segments in the tree for debugging
    RCLCPP_INFO(this->get_logger(), "KDL Tree has %zu segments", tree_.getSegments().size());
    for (const auto& segment_pair : tree_.getSegments())
    {
        RCLCPP_DEBUG(this->get_logger(), "  - Segment: %s", segment_pair.first.c_str());
    }

    RCLCPP_INFO(this->get_logger(), "Attempting to extract chain from '%s' to '%s'", base_link_.c_str(), tip_.c_str());
    bool chain_success = tree_.getChain(base_link_, tip_, chain_);
    if (!chain_success)
    {
        RCLCPP_FATAL(this->get_logger(), "Failed to extract kinematic chain from '%s' to '%s'! These links may not exist or not be connected.", base_link_.c_str(), tip_.c_str());
        rclcpp::shutdown();
        return;
    }
    
    ndof_ = chain_.getNrOfJoints();
    int nseg = chain_.getNrOfSegments();

    RCLCPP_INFO(this->get_logger(), "Loading tree from parameter %s with kinematic chain from %s to %s", robot_description_name.c_str(), base_link_.c_str(), tip_.c_str());
    RCLCPP_INFO(this->get_logger(), "Number of tree segments: %d", tree_.getNrOfSegments());
    RCLCPP_INFO(this->get_logger(), "Number of tree joints: %d", tree_.getNrOfJoints());
    RCLCPP_INFO(this->get_logger(), "Number of chain joints: %d", ndof_);
    RCLCPP_INFO(this->get_logger(), "Number of chain segments: %d", nseg);

    if (ndof_ < 1)
    {
        RCLCPP_FATAL(this->get_logger(), "Robot has 0 joints, check if root and/or tip name is incorrect!");
        rclcpp::shutdown();
    }

    qmax_.resize(ndof_);
    qmin_.resize(ndof_);
    qdotmax_.resize(ndof_);
    qdotmin_.resize(ndof_);
    max_lin_limit_.resize(ndof_);
    min_lin_limit_.resize(ndof_);
    setLinearizationLimit(lin_limit);

    kdl_fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(chain_));
    kdl_dfk_solver_.reset(new KDL::ChainJntToJacSolver(chain_));

    int mvable_jnt(0);
    std::vector<std::string> joint_names(ndof_);
    chain_segment_names_.resize(nseg);  // Pre-allocate for segment names
    
    for (int i = 0; i < nseg; ++i)
    {
        KDL::Segment seg = chain_.getSegment(i);
        KDL::Joint kdl_joint = seg.getJoint();
        urdf::JointConstSharedPtr urdf_joint = model_->getJoint(kdl_joint.getName());
        
        // Store segment name for logging
        chain_segment_names_[i] = seg.getName();

        if (kdl_joint.getType() != KDL::Joint::None)
        {
            // No limits so assign max
            if (urdf_joint->type == urdf::Joint::CONTINUOUS)
            {
                qmax_[mvable_jnt] = 2.0 * M_PI;
                qmin_[mvable_jnt] = -2.0 * M_PI;
            }
            else
            {
                qmax_[mvable_jnt] = model_->joints_.at(kdl_joint.getName())->limits->upper;
                qmin_[mvable_jnt] = model_->joints_.at(kdl_joint.getName())->limits->lower;
            }

            qdotmax_[mvable_jnt] = model_->joints_.at(kdl_joint.getName())->limits->velocity;
            qdotmin_[mvable_jnt] = -model_->joints_.at(kdl_joint.getName())->limits->velocity;
            joint_names[mvable_jnt] = kdl_joint.getName();
            mvable_jnt++;
        }
    }

    // Set active joint states as a parameter
    this->declare_parameter("active_dof", std::vector<std::string>{});
    this->set_parameter(rclcpp::Parameter("active_dof", joint_names));

    ndof_identity_matrix_.resize(ndof_, ndof_);
    ndof_identity_matrix_.setZero();
    for (int i = 0; i < ndof_identity_matrix_.rows(); i++)
    {
        for (int j = 0; j < ndof_identity_matrix_.cols(); j++)
        {
            if (i == j)
            {
                ndof_identity_matrix_(i, j) = 1;
            }
        }
    }

    printVector(qmax_, "qmax_");
    printVector(qmin_, "qmin_");
    printVector(qdotmax_, "qdotmax_");
    printVector(qdotmin_, "qdotmin_");

    // Instantiate ROS services and subscribers/publishers
    mesh_coll_server_ = this->create_service<arm_control::srv::AddRemoveCollisionMesh>(
        "add_remove_collision_mesh",  std::bind(&PathCollisionChecking::addRemoveMeshCallback, this, std::placeholders::_1, std::placeholders::_2));
    solid_coll_server_ = this->create_service<arm_control::srv::AddRemoveCollisionSolid>(
        "add_remove_collision_solid",  std::bind(&PathCollisionChecking::addRemoveSolidCallback, this, std::placeholders::_1, std::placeholders::_2));
    // jacobian_server_ = this->create_service<arm_control::srv::GetJacobianMatrix>(
    //     "get_jacobian_matrix",  std::bind(&PathCollisionChecking::getJacobianCallback, this, std::placeholders::_1, std::placeholders::_2));
    update_pos_server_ = this->create_service<arm_control::srv::UpdateCollisionPose>(
        "update_collision_pose",  std::bind(&PathCollisionChecking::updateCollisionObjectPoseCallback, this, std::placeholders::_1, std::placeholders::_2));
    check_collision_pose_server_ = this->create_service<arm_control::srv::CheckCollisionPose>(
        "check_collision_pose",  std::bind(&PathCollisionChecking::checkCollisionPoseCallback, this, std::placeholders::_1, std::placeholders::_2));

    mesh_coll_server_st_ = this->create_service<arm_control::srv::AddRemoveCollisionMeshStamped>(
    "add_remove_collision_mesh_stamped",
    std::bind(&PathCollisionChecking::addRemoveMeshStampedCallback, this, std::placeholders::_1, std::placeholders::_2));

    solid_coll_server_st_ = this->create_service<arm_control::srv::AddRemoveCollisionSolidStamped>(
    "add_remove_collision_solid_stamped",
    std::bind(&PathCollisionChecking::addRemoveSolidStampedCallback, this, std::placeholders::_1, std::placeholders::_2));

    update_pos_server_st_ = this->create_service<arm_control::srv::UpdateCollisionPoseStamped>(
    "update_collision_pose_stamped",
    std::bind(&PathCollisionChecking::updateCollisionObjectPoseStampedCallback, this, std::placeholders::_1, std::placeholders::_2));

    rclcpp::SubscriptionOptions joint_sub_options;
    joint_sub_options.callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions octo_sub_options;
    octo_sub_options.callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions lin_lim_sub_options;
    lin_lim_sub_options.callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    
    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", rclcpp::QoS(1), 
        std::bind(&PathCollisionChecking::jointStateCallback, this, std::placeholders::_1),
        joint_sub_options);
    octomap_filter_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
        "/octomap_filtered", rclcpp::QoS(1), 
        std::bind(&PathCollisionChecking::octomapCallback, this, std::placeholders::_1),
        octo_sub_options);
    // lin_limit_sub_ = this->create_subscription<std_msgs::msg::Float32>(
    //     "/lin_limit", rclcpp::QoS(1), 
    //     std::bind(&PathCollisionChecking::linLimitCallback, this, std::placeholders::_1),
    //     lin_lim_sub_options);

    // coll_check_timer_ = this->create_wall_timer(
    //         std::chrono::milliseconds(250),
    //         std::bind(&PathCollisionChecking::checkCollisionCallback, this));

    mkr_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker", 1);
    world_obstacles_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("world_obstacles", 1);
    // obj_dist_pub_ = this->create_publisher<arm_control::msg::ObjectDistances>("collision/obj_distances", 1);
    filt_mesh_pub_ = this->create_publisher<octomap_filter_interfaces::msg::FilterMesh>("filter_mesh", 1);
    filt_prim_pub_ = this->create_publisher<octomap_filter_interfaces::msg::FilterPrimitive>("filter_primitive", 1);
    occupancy_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("occupancy_grid", 10);
    occupied_voxels_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("occupied_voxel_centers", rclcpp::QoS(1).transient_local());
    evaluated_voxels_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("evaluated_voxel_centers", rclcpp::QoS(1).transient_local());
    evaluated_joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::QoS(10));

    // Create service clients to forward collision objects to real world
    real_world_mesh_client_ = this->create_client<arm_control::srv::AddRemoveCollisionMesh>(
        "/add_remove_collision_mesh");
    real_world_solid_client_ = this->create_client<arm_control::srv::AddRemoveCollisionSolid>(
        "/add_remove_collision_solid");

    // Wait for real world services to become available (max 10 seconds)
    RCLCPP_INFO(this->get_logger(), "Waiting for real world services...");
    if (real_world_mesh_client_->wait_for_service(std::chrono::seconds(10)))
    {
        RCLCPP_INFO(this->get_logger(), "Real world mesh service is available");
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Real world mesh service not available after 10 seconds");
    }
    
    if (real_world_solid_client_->wait_for_service(std::chrono::seconds(10)))
    {
        RCLCPP_INFO(this->get_logger(), "Real world solid service is available");
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Real world solid service not available after 10 seconds");
    }

    RCLCPP_INFO(this->get_logger(), "Initialized constrained_manipulability");
}

/// ROS interface methods

void PathCollisionChecking::addRemoveMeshCallback(const std::shared_ptr<arm_control::srv::AddRemoveCollisionMesh::Request> req,
                                                      std::shared_ptr<arm_control::srv::AddRemoveCollisionMesh::Response> res)
{
    res->result = false;

    boost::mutex::scoped_lock lock(collision_world_mutex_);
    if (req->remove)
    {
        removeCollisionObject(req->object_id);
    }
    else
    {
        Eigen::Affine3d mesh_T;
        tf2::fromMsg(req->pose, mesh_T);

        robot_collision_checking::FCLObjectPtr obj = std::make_shared<robot_collision_checking::FCLObject>(
            req->mesh, robot_collision_checking::MESH, mesh_T);
        addCollisionObject(obj, req->object_id);
    }

    res->result = true;
    
    // Update visualization after modifying collision world
    publishWorldObstacles();
    
    // Forward request to real world (async call with response callback)
    if (real_world_mesh_client_->service_is_ready())
    {
        auto request = std::make_shared<arm_control::srv::AddRemoveCollisionMesh::Request>(*req);
        
        // Add response callback to verify it succeeded
        auto response_callback = [this, object_id = req->object_id](
            rclcpp::Client<arm_control::srv::AddRemoveCollisionMesh>::SharedFuture future)
        {
            auto response = future.get();
            if (response->result)
            {
                RCLCPP_DEBUG(this->get_logger(), "Real world confirmed mesh ID %d added successfully", object_id);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Real world failed to add mesh ID %d", object_id);
            }
        };
        
        real_world_mesh_client_->async_send_request(request, response_callback);
        RCLCPP_DEBUG(this->get_logger(), "Forwarded mesh request to real world");
    }
    else
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
            "Real world mesh service not available - object only added to collision world");
    }
}

void PathCollisionChecking::addRemoveMeshStampedCallback(
  const std::shared_ptr<arm_control::srv::AddRemoveCollisionMeshStamped::Request> req,
  std::shared_ptr<arm_control::srv::AddRemoveCollisionMeshStamped::Response> res)
{
  res->result = false;
  boost::mutex::scoped_lock lock(collision_world_mutex_);

  if (req->remove) {
    removeCollisionObject(req->object_id);
    res->result = true;
    publishWorldObstacles();
    return;
  }

  // 1) Transform PoseStamped -> base_link_
  geometry_msgs::msg::PoseStamped pose_in_base;
  try {
    pose_in_base = buffer_->transform(req->pose, base_link_, tf2::durationFromSec(0.3));
  } catch (const tf2::TransformException& ex) {
    RCLCPP_ERROR(this->get_logger(), "TF %s -> %s failed: %s",
                 req->pose.header.frame_id.c_str(), base_link_.c_str(), ex.what());
    return;
  }

  // 2) Convert to Eigen
  Eigen::Affine3d base_T_mesh;
  tf2::fromMsg(pose_in_base.pose, base_T_mesh);

  // 3) Create object in collision world (internal frame = base_link_)
  auto obj = std::make_shared<robot_collision_checking::FCLObject>(
      req->mesh, robot_collision_checking::MESH, base_T_mesh);
  addCollisionObject(obj, req->object_id);

  res->result = true;
  publishWorldObstacles();
}

void PathCollisionChecking::addRemoveSolidCallback(const std::shared_ptr<arm_control::srv::AddRemoveCollisionSolid::Request> req,
                                                       std::shared_ptr<arm_control::srv::AddRemoveCollisionSolid::Response> res)
{
    res->result = false;

    boost::mutex::scoped_lock lock(collision_world_mutex_);
    if (req->remove)
    {
        res->result = removeCollisionObject(req->object_id);
        RCLCPP_INFO(this->get_logger(), "Removed collision object ID %d: %s", req->object_id, res->result ? "success" : "not found");
    }
    else
    {
        Eigen::Affine3d solid_T;
        tf2::fromMsg(req->pose, solid_T);

        robot_collision_checking::FCLObjectPtr obj = std::make_shared<robot_collision_checking::FCLObject>(req->solid, solid_T);
        addCollisionObject(obj, req->object_id);
        res->result = true;
        RCLCPP_INFO(this->get_logger(), "Added collision object ID %d (type %d) to collision world", req->object_id, req->solid.type);
    }
    
    // Update visualization after modifying collision world
    publishWorldObstacles();
    
    // Release lock before async service call to prevent potential deadlock
    lock.unlock();
    
    // Forward request to real world (async call with response callback)
    bool service_ready = real_world_solid_client_->service_is_ready();
    RCLCPP_INFO(this->get_logger(), "Real world service ready status: %s", service_ready ? "TRUE" : "FALSE");
    
    if (service_ready)
    {
        auto request = std::make_shared<arm_control::srv::AddRemoveCollisionSolid::Request>(*req);
        
        RCLCPP_INFO(this->get_logger(), "Sending async request for object ID %d to real world service", req->object_id);
        
        // Add response callback to verify it succeeded
        auto response_callback = [this, object_id = req->object_id](
            rclcpp::Client<arm_control::srv::AddRemoveCollisionSolid>::SharedFuture future)
        {
            RCLCPP_INFO(this->get_logger(), "=== Response callback INVOKED for object ID %d ===", object_id);
            try
            {
                auto response = future.get();
                if (response->result)
                {
                    RCLCPP_INFO(this->get_logger(), "Real world confirmed object ID %d added successfully", object_id);
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "Real world failed to add object ID %d", object_id);
                }
            }
            catch (const std::exception& e)
            {
                RCLCPP_ERROR(this->get_logger(), "Exception in response callback for object ID %d: %s", object_id, e.what());
            }
        };
        
        real_world_solid_client_->async_send_request(request, response_callback);
        RCLCPP_INFO(this->get_logger(), "Async request dispatched for object ID %d to real world", req->object_id);
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), 
            "Real world solid service NOT READY - object ID %d only added to collision world", req->object_id);
    }
}


void PathCollisionChecking::addRemoveSolidStampedCallback(
  const std::shared_ptr<arm_control::srv::AddRemoveCollisionSolidStamped::Request> req,
  std::shared_ptr<arm_control::srv::AddRemoveCollisionSolidStamped::Response> res)
{
  res->result = false;
  boost::mutex::scoped_lock lock(collision_world_mutex_);

  if (req->remove) {
    removeCollisionObject(req->object_id);
    res->result = true;
    publishWorldObstacles();
    return;
  }

  geometry_msgs::msg::PoseStamped pose_in_base;
  try {
    pose_in_base = buffer_->transform(req->pose, base_link_, tf2::durationFromSec(0.3));
  } catch (const tf2::TransformException& ex) {
    RCLCPP_ERROR(this->get_logger(), "TF %s -> %s failed: %s",
                 req->pose.header.frame_id.c_str(), base_link_.c_str(), ex.what());
    return;
  }

  Eigen::Affine3d base_T_solid;
  tf2::fromMsg(pose_in_base.pose, base_T_solid);

  auto obj = std::make_shared<robot_collision_checking::FCLObject>(req->solid, base_T_solid);
  addCollisionObject(obj, req->object_id);

  res->result = true;
  publishWorldObstacles();
}

void PathCollisionChecking::updateCollisionObjectPoseCallback(const std::shared_ptr<arm_control::srv::UpdateCollisionPose::Request> req,
                                                       std::shared_ptr<arm_control::srv::UpdateCollisionPose::Response> res)
{
    Eigen::Affine3d new_pose;
    tf2::fromMsg(req->pose, new_pose);
    res->result = collision_world_->updateCollisionObjectPose(req->object_id, new_pose);
}


void PathCollisionChecking::updateCollisionObjectPoseStampedCallback(
  const std::shared_ptr<arm_control::srv::UpdateCollisionPoseStamped::Request> req,
  std::shared_ptr<arm_control::srv::UpdateCollisionPoseStamped::Response> res)
{
  geometry_msgs::msg::PoseStamped pose_in_base;
  try {
    pose_in_base = buffer_->transform(req->pose, base_link_, tf2::durationFromSec(0.3));
  } catch (const tf2::TransformException& ex) {
    RCLCPP_ERROR(this->get_logger(), "TF %s -> %s failed: %s",
                 req->pose.header.frame_id.c_str(), base_link_.c_str(), ex.what());
    res->result = false;
    return;
  }

  Eigen::Affine3d base_T_obj;
  tf2::fromMsg(pose_in_base.pose, base_T_obj);
  res->result = collision_world_->updateCollisionObjectPose(req->object_id, base_T_obj);
}

void PathCollisionChecking::checkCollisionPoseCallback(const std::shared_ptr<arm_control::srv::CheckCollisionPose::Request> req,
                                                        std::shared_ptr<arm_control::srv::CheckCollisionPose::Response> res)
{
    try 
    {
        sensor_msgs::msg::JointState prefixed_state = convertToNamespaceJointState(req->joint_state);

        bool self_col, env_col;
        res->in_collision = evaluateCollisionState(prefixed_state, self_col, env_col, req->publish_visualization);
        
        // Always publish joint states so the collision robot visualization stays at the tested configuration
        evaluated_joint_pub_->publish(prefixed_state);
    } 
    catch (const std::exception& e) 
    {
        RCLCPP_ERROR(this->get_logger(), "Error in checkCollisionPoseCallback: %s", e.what());
        res->in_collision = false;  // opción conservadora
    }
    catch (...)
    {
        RCLCPP_ERROR(this->get_logger(), "Unknown error in checkCollisionPoseCallback");
        res->in_collision = false;
    }
}

sensor_msgs::msg::JointState PathCollisionChecking::convertToNamespaceJointState(const sensor_msgs::msg::JointState& input) const
{
    sensor_msgs::msg::JointState visual;
    visual.header.stamp = this->now();
    visual.position = input.position;
    visual.velocity = input.velocity;
    visual.effort = input.effort;
    for (const auto& name : input.name)
        visual.name.push_back("collision_" + name);
    return visual;
}


// void PathCollisionChecking::getJacobianCallback(const std::shared_ptr<arm_control::srv::GetJacobianMatrix::Request> req,
//                                                     std::shared_ptr<arm_control::srv::GetJacobianMatrix::Response> res)
// {
//     Eigen::Matrix<double, 6, Eigen::Dynamic> base_J_ee;
//     getJacobian(req->joint_state, base_J_ee);
//     res->jacobian = eigenToMatrix(base_J_ee);
// }

void PathCollisionChecking::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    joint_state_mutex_.lock();
    joint_state_ = *msg;
    KDL::JntArray kdl_joint_positions(ndof_);
    jointStatetoKDLJointArray(chain_, joint_state_, kdl_joint_positions);
    joint_state_mutex_.unlock();

    GeometryInformation geometry_information;
    getCollisionModel(kdl_joint_positions, geometry_information);
    std::vector<std::string> branch_link_names = addBranchCollisionGeometry(kdl_joint_positions, geometry_information);

    std::vector<shapes::ShapeMsg> current_shapes;
    std::vector<geometry_msgs::msg::Pose> shapes_poses;
    convertCollisionModel(geometry_information, current_shapes, shapes_poses);

    if (filter_robot_)
    {
        int num_shapes = current_shapes.size();
        for (int i = 0; i < num_shapes; i++)
        {
            geometry_msgs::msg::PoseStamped shape_stamped;
            shape_stamped.header.stamp = msg->header.stamp;
            shape_stamped.header.frame_id = base_link_;
            shape_stamped.pose = shapes_poses[i];
            // Filter robot from octomap
            if (current_shapes[i].which() == 0)
            {
                octomap_filter_interfaces::msg::FilterPrimitive filter_primitive;
                filter_primitive.primitive = boost::get<shape_msgs::msg::SolidPrimitive>(current_shapes[i]);
                filter_primitive.pose = shape_stamped;
                filt_prim_pub_->publish(filter_primitive);
            }
            else if (current_shapes[i].which() == 1)
            {
                octomap_filter_interfaces::msg::FilterMesh filter_mesh;
                filter_mesh.mesh = boost::get<shape_msgs::msg::Mesh>(current_shapes[i]);
                filter_mesh.pose = shape_stamped;
                filt_mesh_pub_->publish(filter_mesh);
            }
        }
    }

    displayCollisionModel(geometry_information, {0.1, 0.5, 0.2, 0.5});
 }

void PathCollisionChecking::octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg)
{
    // Get octomap pose w.r.t. the robot base frame
    geometry_msgs::msg::TransformStamped octomap_wrt_base;
    try
    {
        octomap_wrt_base = buffer_->lookupTransform(
            base_link_,
            msg->header.frame_id,
            tf2::TimePointZero);
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_ERROR(this->get_logger(), "Could not transform %s to %s: %s",
            base_link_.c_str(), msg->header.frame_id.c_str(), ex.what());                
    }

    Eigen::Affine3d octomap_pose_wrt_base = tf2::transformToEigen(octomap_wrt_base.transform);

    boost::mutex::scoped_lock lock(collision_world_mutex_);
    // Remove the old octomap from the world
    collision_world_->removeCollisionObject(OCTOMAP_ID);
    // Update with the new octomap
    robot_collision_checking::FCLObjectPtr octo_obj = std::make_shared<robot_collision_checking::FCLObject>(
        *msg, robot_collision_checking::OCTOMAP, octomap_pose_wrt_base);
    // Add the filtered octomap to the collision world
    collision_world_->addCollisionObject(octo_obj, OCTOMAP_ID);
}

// void PathCollisionChecking::generateOccupancyGrid()
// {
//     boost::mutex::scoped_lock lock(collision_world_mutex_);

//     const double resolution = 0.05;  // tamaño del voxel
//     const double half_res = resolution / 2.0;

//     std::set<std::tuple<int, int, int>> occupied_cells;
//     std::vector<Eigen::Vector3d> all_voxel_centers;

//     for (const auto& obj : collision_world_->getCollisionObjects())
//     {
//         if (!obj)
//             continue;

//         const fcl::AABBd& aabb = obj->collision_object->getAABB();

//         double x_start = std::floor(aabb.min_.x() / resolution) * resolution - half_res;
//         double y_start = std::floor(aabb.min_.y() / resolution) * resolution - half_res;
//         double z_start = std::floor(aabb.min_.z() / resolution) * resolution - half_res;

//         double x_end = std::ceil(aabb.max_.x() / resolution) * resolution + half_res;
//         double y_end = std::ceil(aabb.max_.y() / resolution) * resolution + half_res;
//         double z_end = std::ceil(aabb.max_.z() / resolution) * resolution + half_res;

//         for (double x = x_start; x <= x_end; x += resolution)
//         {
//             for (double y = y_start; y <= y_end; y += resolution)
//             {
//                 for (double z = z_start; z <= z_end; z += resolution)
//                 {
//                     Eigen::Vector3d voxel_center(x + half_res, y + half_res, z + half_res);
//                     all_voxel_centers.push_back(voxel_center);
//                     fcl::Transform3d tf = fcl::Transform3d::Identity();
//                     tf.translation() = voxel_center;

//                     auto voxel_shape = std::make_shared<fcl::Boxd>(resolution, resolution, resolution);
//                     fcl::CollisionObjectd voxel_box(voxel_shape, tf);

//                     fcl::DistanceRequestd req(true);  // enable signed distance
//                     fcl::DistanceResultd res;
//                     double distance = fcl::distance(&voxel_box, obj->collision_object.get(), req, res);

//                     // Tolerancia mayor para considerar interiores
//                     if (distance < resolution * 0.75)  // por ejemplo 75% del tamaño del voxel
//                     {
//                         int xi = static_cast<int>(std::floor(x / resolution));
//                         int yi = static_cast<int>(std::floor(y / resolution));
//                         int zi = static_cast<int>(std::floor(z / resolution));
//                         occupied_cells.emplace(xi, yi, zi);
//                     }
//                     if (distance > 0.5 * resolution && distance < 1.5 * resolution)
//                     {
//                         // RCLCPP_INFO(this->get_logger(), "Near miss at voxel (%.2f, %.2f, %.2f), distance = %.4f",
//                         //             voxel_center.x(), voxel_center.y(), voxel_center.z(), distance);
//                     }
//                 }
//             }
//         }
//     }

//     // RCLCPP_INFO(this->get_logger(), "Total occupied voxels: %zu", occupied_cells.size());
//     publishVoxelMarkers(occupied_cells, resolution);

//     sensor_msgs::msg::PointCloud2 cloud_msg;
//     cloud_msg.header.stamp = this->now();
//     cloud_msg.header.frame_id = "base_link";  // o el frame correcto

//     sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
//     modifier.setPointCloud2FieldsByString(1, "xyz");
//     modifier.resize(occupied_cells.size());

//     sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
//     sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
//     sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

//     for (const auto& cell : occupied_cells) {
//         geometry_msgs::msg::Point32 pt;
//         pt.x = (static_cast<float>(std::get<0>(cell)) + 0.5f) * resolution;
//         pt.y = (static_cast<float>(std::get<1>(cell)) + 0.5f) * resolution;
//         pt.z = (static_cast<float>(std::get<2>(cell)) + 0.5f) * resolution;
//         *iter_x = pt.x;
//         *iter_y = pt.y;
//         *iter_z = pt.z;
//         ++iter_x;
//         ++iter_y;
//         ++iter_z;
//     }

//     occupied_voxels_pub_->publish(cloud_msg);

//     sensor_msgs::msg::PointCloud2 eval_cloud;
//     eval_cloud.header.stamp = this->now();
//     eval_cloud.header.frame_id = "base_link";

//     sensor_msgs::PointCloud2Modifier eval_modifier(eval_cloud);
//     eval_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
//     eval_modifier.resize(all_voxel_centers.size());

//     sensor_msgs::PointCloud2Iterator<float> ex(eval_cloud, "x");
//     sensor_msgs::PointCloud2Iterator<float> ey(eval_cloud, "y");
//     sensor_msgs::PointCloud2Iterator<float> ez(eval_cloud, "z");
//     sensor_msgs::PointCloud2Iterator<uint8_t> er(eval_cloud, "rgb");

//     for (const auto& pt : all_voxel_centers) {
//         *ex = pt.x();
//         *ey = pt.y();
//         *ez = pt.z();
//         uint8_t r = 255, g = 0, b = 0;
//         uint32_t rgb = (r << 16) | (g << 8) | b;
//         *er = *reinterpret_cast<float*>(&rgb);
//         ++ex; ++ey; ++ez; ++er;
//     }

//     evaluated_voxels_pub_->publish(eval_cloud);

// }

// void PathCollisionChecking::publishVoxelMarkers(const std::set<std::tuple<int, int, int>>& occupied_cells, double resolution)
// {
//     visualization_msgs::msg::MarkerArray marker_array;
//     int id = 0;

//     for (const auto& [xi, yi, zi] : occupied_cells)
//     {
//         visualization_msgs::msg::Marker marker;
//         marker.header.frame_id = "world";  // usa tu frame adecuado
//         marker.header.stamp = this->now();
//         marker.ns = "occupied_voxels";
//         marker.id = id++;
//         marker.type = visualization_msgs::msg::Marker::CUBE;
//         marker.action = visualization_msgs::msg::Marker::ADD;
//         marker.pose.position.x = (xi + 0.5) * resolution;
//         marker.pose.position.y = (yi + 0.5) * resolution;
//         marker.pose.position.z = (zi + 0.5) * resolution;
//         marker.pose.orientation.w = 1.0;
//         marker.scale.x = resolution;
//         marker.scale.y = resolution;
//         marker.scale.z = resolution;
//         marker.color.r = 0.0f;
//         marker.color.g = 0.8f;
//         marker.color.b = 0.1f;
//         marker.color.a = 0.5f;
//         marker.lifetime = rclcpp::Duration::from_seconds(1.0);  // permanente

//         marker_array.markers.push_back(marker);
//     }

//     occupancy_pub_->publish(marker_array);
// }


// void PathCollisionChecking::linLimitCallback(const std_msgs::msg::Float32::SharedPtr msg)
// {
//     setLinearizationLimit(msg->data);    
// }

// void PathCollisionChecking::checkCollisionCallback()
// {
//     // joint_state_mutex_.lock();
//     // sensor_msgs::msg::JointState curr_joint_state = joint_state_;
//     // joint_state_mutex_.unlock();

//     // // checkCollision(curr_joint_state);    // opcion original

//     // bool collision_detected = checkCollision(curr_joint_state);  // opcion propia
//     // if (collision_detected)
//     // {
//     //     RCLCPP_WARN(this->get_logger(), "¡Colisión detectada!");
//     // }
//     // else
//     // {
//     //     RCLCPP_WARN(this->get_logger(), "Sin colisión");
//     // }  

//     // static bool last_collision_state = false;
//     // rclcpp::Logger logger = this->get_logger();

//     // joint_state_mutex_.lock();
//     // sensor_msgs::msg::JointState curr_joint_state = joint_state_;
//     // joint_state_mutex_.unlock();
    
//     // bool collision_detected = checkCollision(curr_joint_state);

//     // if (collision_detected != last_collision_state)
//     // {
//     //     last_collision_state = collision_detected;
//     //     if (collision_detected)
//     //     {
//     //         RCLCPP_WARN(logger, "¡Colisión detectada!");
//     //     }
//     //     else
//     //     {
//     //         RCLCPP_WARN(logger, "Sin colisión");
//     //     }
//     // }

//     joint_state_mutex_.lock();
//     sensor_msgs::msg::JointState curr_joint_state = joint_state_;
//     joint_state_mutex_.unlock();


//     bool collision_detected = false;

//     try
//     {
//         collision_detected = checkCollision(curr_joint_state);
//     }
//     catch (const std::exception& e)
//     {
//         RCLCPP_ERROR(this->get_logger(), "Exception in checkCollision: %s", e.what());
//         return;
//     }
//     catch (...)
//     {
//         RCLCPP_ERROR(this->get_logger(), "Unknown exception in checkCollision");
//         return;
//     }

//     if (collision_detected != last_collision_state_)
//     {
//         last_collision_state_ = collision_detected;
//         if (collision_detected)
//         {
//             RCLCPP_WARN(this->get_logger(), "Collision detected!");
//         }
//         else
//         {
//             RCLCPP_INFO(this->get_logger(), "All collisions cleared");
//         }
//     }
// }

/// Private member methods (excluding ROS callbacks)

bool PathCollisionChecking::addCollisionObject(const robot_collision_checking::FCLObjectPtr& obj, int object_id)
{
    return collision_world_->addCollisionObject(obj, object_id);
}

bool PathCollisionChecking::removeCollisionObject(int object_id)
{
    return collision_world_->removeCollisionObject(object_id);
}

// bool PathCollisionChecking::checkCollision(const sensor_msgs::msg::JointState& joint_state)
// {
//     KDL::JntArray kdl_joint_positions(ndof_);
//     jointStatetoKDLJointArray(chain_, joint_state, kdl_joint_positions);

//     GeometryInformation geometry_information;
//     getCollisionModel(kdl_joint_positions, geometry_information);

//     std::vector<shapes::ShapeMsg> current_shapes;
//     std::vector<geometry_msgs::msg::Pose> shapes_poses;
//     convertCollisionModel(geometry_information, current_shapes, shapes_poses);

//     // Build collision geometry for robot if not yet created
//     // Assume collision geometry for robot's meshes do not change
//     if (robot_collision_geometry_.size() == 0)
//     {
//         createRobotCollisionModel(geometry_information);
//     }
    
//     boost::mutex::scoped_lock lock(collision_world_mutex_);

//     // Self-collision check
//     bool self_collision_detected = checkSelfCollision(geometry_information);
//     if (self_collision_detected != last_self_collision_state_)
//     {
//         last_self_collision_state_ = self_collision_detected;
//         if (self_collision_detected)
//         {
//             RCLCPP_WARN(this->get_logger(), "Self-collision detected!");
//         }
//         else
//         {
//             RCLCPP_INFO(this->get_logger(), "Self-collision cleared.");
//         }
//     }

//     // External environement collision check
//     bool env_collision_detected = false;
//     int num_shapes = geometry_information.shapes.size();
//     for (int i = 0; i < num_shapes; i++)
//     {
//         Eigen::Affine3d obj_pose;
//         tf2::fromMsg(shapes_poses[i], obj_pose);
//         std::vector<int> collision_object_ids;
//         if (current_shapes[i].which() == 0)
//         {
//             robot_collision_checking::FCLObjectPtr obj = std::make_shared<robot_collision_checking::FCLObject>(
//                 boost::get<shape_msgs::msg::SolidPrimitive>(current_shapes[i]), obj_pose);

//             fcl::Transform3d world_to_fcl;
//             robot_collision_checking::fcl_interface::transform2fcl(obj->object_transform, world_to_fcl);
//             robot_collision_checking::FCLCollisionObjectPtr co = std::make_shared<fcl::CollisionObjectd>(robot_collision_geometry_[i], world_to_fcl);
            
//             if (collision_world_->checkCollisionObject(co, collision_object_ids))
//             {
//                 env_collision_detected = true;
//             }
//         }
//         else if (current_shapes[i].which() == 1)
//         {
//             robot_collision_checking::FCLObjectPtr obj = std::make_shared<robot_collision_checking::FCLObject>(
//                 boost::get<shape_msgs::msg::Mesh>(current_shapes[i]), robot_collision_checking::MESH, obj_pose);

//             fcl::Transform3d world_to_fcl;
//             robot_collision_checking::fcl_interface::transform2fcl(obj->object_transform, world_to_fcl);
//             robot_collision_checking::FCLCollisionObjectPtr co = std::make_shared<fcl::CollisionObjectd>(robot_collision_geometry_[i], world_to_fcl);

//             if (collision_world_->checkCollisionObject(co, collision_object_ids))
//             {
//                 env_collision_detected = true;
//             }
//         }
//         else
//         {
//             RCLCPP_ERROR(this->get_logger(), "Collision geometry not supported");
//         }
//     }

//     if (env_collision_detected != last_env_collision_state_)
//     {
//         last_env_collision_state_ = env_collision_detected;
//         if (env_collision_detected)
//         {
//             RCLCPP_WARN(this->get_logger(), "Environement collision detected!");
//         }
//         else
//         {
//             RCLCPP_INFO(this->get_logger(), "Environement collision cleared.");
//         }
//     }
    
//     return env_collision_detected || self_collision_detected;
// }

bool PathCollisionChecking::evaluateCollisionState(const sensor_msgs::msg::JointState& joint_state, bool& self_collision, bool& env_collision, bool publish_visualization)
{
    RCLCPP_INFO(this->get_logger(), "=== evaluateCollisionState START ===");
    KDL::JntArray kdl_joint_positions(ndof_);
    jointStatetoKDLJointArray(chain_, joint_state, kdl_joint_positions);

    RCLCPP_INFO(this->get_logger(), "Getting collision model...");
    GeometryInformation geometry_information;
    getCollisionModel(kdl_joint_positions, geometry_information);
    std::vector<std::string> branch_link_names = addBranchCollisionGeometry(kdl_joint_positions, geometry_information);
    
    RCLCPP_INFO(this->get_logger(), "Collision model retrieved: %zu shapes (%zu from branches), %zu transforms, %zu jacobians",
                geometry_information.shapes.size(), branch_link_names.size(),
                geometry_information.geometry_transforms.size(), 
                geometry_information.geometry_jacobians.size());

    std::vector<shapes::ShapeMsg> current_shapes;
    std::vector<geometry_msgs::msg::Pose> shapes_poses;
    RCLCPP_INFO(this->get_logger(), "Converting collision model...");
    convertCollisionModel(geometry_information, current_shapes, shapes_poses);
    RCLCPP_INFO(this->get_logger(), "Converted to %zu shapes", current_shapes.size());

    if (robot_collision_geometry_.size() == 0)
    {
        RCLCPP_INFO(this->get_logger(), "Creating robot collision model for the first time...");
        createRobotCollisionModel(geometry_information, branch_link_names);
        RCLCPP_INFO(this->get_logger(), "Created %zu collision geometries", robot_collision_geometry_.size());
        
        // Build collision exclusions for adjacent and fixed links
        buildCollisionExclusions();
    }
    
    // Validate sizes match
    if (robot_collision_geometry_.size() != geometry_information.shapes.size())
    {
        RCLCPP_ERROR(this->get_logger(), 
                     "Size mismatch: robot_collision_geometry_ has %zu elements but geometry_information has %zu shapes. Recreating collision model.",
                     robot_collision_geometry_.size(), geometry_information.shapes.size());
        robot_collision_geometry_.clear();
        createRobotCollisionModel(geometry_information, branch_link_names);
        buildCollisionExclusions();
    }

    RCLCPP_INFO(this->get_logger(), "Acquiring collision world mutex...");
    boost::mutex::scoped_lock lock(collision_world_mutex_);

    RCLCPP_INFO(this->get_logger(), "Checking self collision...");
    self_collision = checkSelfCollision(geometry_information);
    RCLCPP_INFO(this->get_logger(), "Self collision check complete: %s", self_collision ? "COLLISION" : "clear");

    // Display collision model if visualization requested
    if (publish_visualization) {
        displayCollisionModel(geometry_information, {0.1, 0.8, 0.1, 0.6});  // Green with transparency
    }

    env_collision = false;
    int num_shapes = geometry_information.shapes.size();
    for (int i = 0; i < num_shapes; i++)
    {
        Eigen::Affine3d obj_pose;
        tf2::fromMsg(shapes_poses[i], obj_pose);
        std::vector<int> collision_object_ids;

        fcl::Transform3d world_to_fcl;
        robot_collision_checking::FCLCollisionObjectPtr co;

        if (current_shapes[i].which() == 0)
        {
            auto obj = std::make_shared<robot_collision_checking::FCLObject>(
                boost::get<shape_msgs::msg::SolidPrimitive>(current_shapes[i]), obj_pose);
            robot_collision_checking::fcl_interface::transform2fcl(obj->object_transform, world_to_fcl);
            co = std::make_shared<fcl::CollisionObjectd>(robot_collision_geometry_[i], world_to_fcl);
        }
        else if (current_shapes[i].which() == 1)
        {
            auto obj = std::make_shared<robot_collision_checking::FCLObject>(
                boost::get<shape_msgs::msg::Mesh>(current_shapes[i]), robot_collision_checking::MESH, obj_pose);
            robot_collision_checking::fcl_interface::transform2fcl(obj->object_transform, world_to_fcl);
            co = std::make_shared<fcl::CollisionObjectd>(robot_collision_geometry_[i], world_to_fcl);
        }
        else
        {
            continue;
        }

        if (collision_world_->checkCollisionObject(co, collision_object_ids))
        {
            env_collision = true;
            break;
        }
    }

    return self_collision || env_collision;
}


bool PathCollisionChecking::checkSelfCollision(const GeometryInformation& geometry_information)
{
    bool any_collision_active = false;
    int n = geometry_information.shapes.size();
    int n_names = static_cast<int>(collision_link_names_.size());
    int n_geom = static_cast<int>(robot_collision_geometry_.size());

    // Critical validation before accessing vectors
    if (n != n_geom)
    {
        RCLCPP_ERROR(this->get_logger(), 
                     "CRITICAL: Shapes count (%d) != robot_collision_geometry_ count (%d). Cannot check collision!", 
                     n, n_geom);
        return false;
    }

    if (n != n_names)
    {
        RCLCPP_WARN(this->get_logger(), 
                    "Mismatch: %d shapes but %d link names. Using indices only.", n, n_names);
    }

    for (int i = 0; i < n; ++i)
    {
        // Double-check index validity (defensive programming)
        if (i >= n_geom)
        {
            RCLCPP_ERROR(this->get_logger(), "Index %d out of bounds for robot_collision_geometry_ (size %d)", i, n_geom);
            return false;
        }

        Eigen::Isometry3d pose_i;
        pose_i.linear() = geometry_information.geometry_transforms[i].linear();
        pose_i.translation() = geometry_information.geometry_transforms[i].translation();
        fcl::CollisionObjectd obj_i(robot_collision_geometry_[i], pose_i);

        for (int j = i + 1; j < n; ++j)
        {
            // Skip adjacent pairs
            if (isAdjacent(i, j)) continue;

            Eigen::Isometry3d pose_j;
            pose_j.linear() = geometry_information.geometry_transforms[j].linear();
            pose_j.translation() = geometry_information.geometry_transforms[j].translation();
            fcl::CollisionObjectd obj_j(robot_collision_geometry_[j], pose_j);

            fcl::CollisionRequestd request;
            fcl::CollisionResultd result;

            fcl::collide(&obj_i, &obj_j, request, result);

            bool collision_detected = result.isCollision();
            auto key = std::make_pair(i, j);
            if (collision_detected != links_collision_states_[key])
            {
                links_collision_states_[key] = collision_detected;
                if (collision_detected)
                {
                    // Get link names for better debugging - safe access to pre-built collision_link_names_
                    std::string link_i_name = (i < n_names) ? collision_link_names_[i] : "link_" + std::to_string(i);
                    std::string link_j_name = (j < n_names) ? collision_link_names_[j] : "link_" + std::to_string(j);
                    
                    RCLCPP_WARN(this->get_logger(), "Self-collision detected between '%s' (index %d) and '%s' (index %d)", 
                                link_i_name.c_str(), i, link_j_name.c_str(), j);
                    return true;
                }
                else
                {
                    std::string link_i_name = (i < n_names) ? collision_link_names_[i] : "link_" + std::to_string(i);
                    std::string link_j_name = (j < n_names) ? collision_link_names_[j] : "link_" + std::to_string(j);
                    
                    RCLCPP_INFO(this->get_logger(), "Self-collision cleared between '%s' (index %d) and '%s' (index %d)", 
                                link_i_name.c_str(), i, link_j_name.c_str(), j);
                }
            }
            // Registrar si hay alguna colisión activa
            if (collision_detected)
            {
                any_collision_active = true;
            }
        }
    }

    return any_collision_active;
}

bool PathCollisionChecking::isAdjacent(int i, int j) const
{
    // Check if pair is in the exclusion set
    auto key1 = std::make_pair(i, j);
    auto key2 = std::make_pair(j, i);  // Check both orders
    return (collision_exclusions_.find(key1) != collision_exclusions_.end()) ||
           (collision_exclusions_.find(key2) != collision_exclusions_.end());
}

void PathCollisionChecking::buildCollisionExclusions()
{
    collision_exclusions_.clear();
    
    // Find which index corresponds to each link name
    std::map<std::string, int> link_name_to_index;
    for (size_t i = 0; i < collision_link_names_.size(); ++i)
    {
        link_name_to_index[collision_link_names_[i]] = i;
    }
    
    // 1. Add adjacent chain links (consecutive in the kinematic chain)
    for (int i = 0; i < static_cast<int>(chain_.getNrOfSegments()) - 1; ++i)
    {
        std::string link_i = chain_.getSegment(i).getName();
        std::string link_j = chain_.getSegment(i + 1).getName();
        
        // Find indices in collision_link_names_
        auto it_i = link_name_to_index.find(link_i);
        auto it_j = link_name_to_index.find(link_j);
        
        if (it_i != link_name_to_index.end() && it_j != link_name_to_index.end())
        {
            int idx_i = it_i->second;
            int idx_j = it_j->second;
            collision_exclusions_.insert(std::make_pair(std::min(idx_i, idx_j), std::max(idx_i, idx_j)));
            RCLCPP_DEBUG(this->get_logger(), "Excluding adjacent chain links: %s (%d) <-> %s (%d)", 
                        link_i.c_str(), idx_i, link_j.c_str(), idx_j);
        }
    }
    
    // 2. Add parent-child relationships for all links (especially branches)
    // Build parent map from URDF joints
    std::map<std::string, std::string> child_to_parent;
    for (const auto& joint_pair : model_->joints_)
    {
        const urdf::JointSharedPtr& joint = joint_pair.second;
        // Skip if this is not a fixed joint (we only care about rigidly connected links)
        if (joint->type == urdf::Joint::FIXED)
        {
            child_to_parent[joint->child_link_name] = joint->parent_link_name;
        }
    }
    
    // For each link with collision geometry, exclude it from its parent if fixed joint
    for (const auto& link_name : collision_link_names_)
    {
        auto parent_it = child_to_parent.find(link_name);
        if (parent_it != child_to_parent.end())
        {
            const std::string& parent_name = parent_it->second;
            auto parent_idx_it = link_name_to_index.find(parent_name);
            auto child_idx_it = link_name_to_index.find(link_name);
            
            if (parent_idx_it != link_name_to_index.end() && child_idx_it != link_name_to_index.end())
            {
                int parent_idx = parent_idx_it->second;
                int child_idx = child_idx_it->second;
                collision_exclusions_.insert(std::make_pair(std::min(parent_idx, child_idx), std::max(parent_idx, child_idx)));
                RCLCPP_DEBUG(this->get_logger(), "Excluding fixed parent-child: %s (%d) <-> %s (%d)", 
                            parent_name.c_str(), parent_idx, link_name.c_str(), child_idx);
            }
        }
    }
    
    // 3. Exclude siblings (links with the same parent via fixed joints) - they're often close/overlapping
    std::map<std::string, std::vector<std::string>> parent_to_children;
    for (const auto& pair : child_to_parent)
    {
        parent_to_children[pair.second].push_back(pair.first);
    }
    
    for (const auto& parent_children_pair : parent_to_children)
    {
        const std::vector<std::string>& siblings = parent_children_pair.second;
        // Exclude all pairs of siblings
        for (size_t i = 0; i < siblings.size(); ++i)
        {
            for (size_t j = i + 1; j < siblings.size(); ++j)
            {
                auto idx_i_it = link_name_to_index.find(siblings[i]);
                auto idx_j_it = link_name_to_index.find(siblings[j]);
                
                if (idx_i_it != link_name_to_index.end() && idx_j_it != link_name_to_index.end())
                {
                    int idx_i = idx_i_it->second;
                    int idx_j = idx_j_it->second;
                    collision_exclusions_.insert(std::make_pair(std::min(idx_i, idx_j), std::max(idx_i, idx_j)));
                    RCLCPP_DEBUG(this->get_logger(), "Excluding siblings: %s (%d) <-> %s (%d)", 
                                siblings[i].c_str(), idx_i, siblings[j].c_str(), idx_j);
                }
            }
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "Built %zu collision exclusion pairs", collision_exclusions_.size());
}

void PathCollisionChecking::displayCollisionModel(const GeometryInformation& geometry_information, const Eigen::Vector4d& color)
{
    // Publish robot collision geometry (green mesh)
    auto marker_array_msg = std::make_shared<visualization_msgs::msg::MarkerArray>();

    int num_shapes = geometry_information.shapes.size();
    for (int i = 0; i < num_shapes; ++i)
    {
        visualization_msgs::msg::Marker mkr;
        shapes::constructMarkerFromShape(geometry_information.shapes[i].get(), mkr);
        mkr.ns = "collision_body";
        mkr.header.frame_id = base_link_;
        mkr.action = visualization_msgs::msg::Marker::ADD;
        mkr.lifetime = rclcpp::Duration(0, 0);
        mkr.id = i;
        mkr.color.r = color(0);
        mkr.color.g = color(1);
        mkr.color.b = color(2);
        mkr.color.a = color(3);

        Eigen::Quaterniond q(geometry_information.geometry_transforms[i].linear());
        mkr.pose.position.x = geometry_information.geometry_transforms[i](0, 3);
        mkr.pose.position.y = geometry_information.geometry_transforms[i](1, 3);
        mkr.pose.position.z = geometry_information.geometry_transforms[i](2, 3);
        mkr.pose.orientation.w = q.w();
        mkr.pose.orientation.x = q.x();
        mkr.pose.orientation.y = q.y();
        mkr.pose.orientation.z = q.z();

        // Safety check: ensure scale is never zero (RViz warning for invalid scale)
        if (mkr.scale.x <= 0.0) mkr.scale.x = 0.01;
        if (mkr.scale.y <= 0.0) mkr.scale.y = 0.01;
        if (mkr.scale.z <= 0.0) mkr.scale.z = 0.01;
        
        // Debug log for problematic markers
        if (mkr.scale.x <= 0.001 || mkr.scale.y <= 0.001 || mkr.scale.z <= 0.001)
        {
            std::string link_name = (i < static_cast<int>(collision_link_names_.size())) ? 
                                    collision_link_names_[i] : "unknown_link_" + std::to_string(i);
            RCLCPP_WARN(this->get_logger(), "Marker %d (%s) has very small scale: [%.3f, %.3f, %.3f]", 
                        i, link_name.c_str(), mkr.scale.x, mkr.scale.y, mkr.scale.z);
        }

        marker_array_msg->markers.push_back(mkr);
    }

    mkr_pub_->publish(*marker_array_msg);
    
    // Publish world obstacles on separate topic
    publishWorldObstacles();

    // generateOccupancyGrid();
}

void PathCollisionChecking::publishWorldObstacles()
{
    auto marker_array_msg = std::make_shared<visualization_msgs::msg::MarkerArray>();
    
    std::vector<robot_collision_checking::FCLInterfaceCollisionObjectPtr> world_objects = collision_world_->getCollisionObjects();
    int num_objects = collision_world_->getNumObjects();

    for (int i = 0; i < num_objects; /*i++*/)
    {
        auto world_obj = world_objects[i];
        visualization_msgs::msg::Marker mkr;
        mkr.ns = "collision_objects";
        mkr.header.frame_id = base_link_;
        mkr.action = visualization_msgs::msg::Marker::ADD;
        mkr.lifetime = rclcpp::Duration(0, 0);
        std::string obj_type = world_obj->object->getTypeString();

        // Get object pose relative to world_frame
        Eigen::Affine3d object_eig_pose = world_obj->object->object_transform;
        geometry_msgs::msg::Pose object_geo_pose;
        robot_collision_checking::fcl_interface::convertEigenTransformGeometryPose(object_eig_pose, object_geo_pose);
        mkr.pose = object_geo_pose;

        if (obj_type == "MESH")
        {
            geometric_shapes::constructMarkerFromShape(*(world_obj->object->ptr.mesh), mkr);
            mkr.id = world_obj->collision_id;
            mkr.color.b = 1.0;
            mkr.color.a = 1.0;
            marker_array_msg->markers.push_back(mkr);
            i++;
        }
        else if (obj_type == "PLANE")
        {
            mkr.scale.x = 10.0;
            mkr.scale.y = 10.0;
            mkr.scale.z = 0.001;
            mkr.type = visualization_msgs::msg::Marker::CUBE;
            mkr.id = world_obj->collision_id;
            mkr.color.r = 1.0;
            mkr.color.a = 0.3;
            marker_array_msg->markers.push_back(mkr);
            i++;
        }
        else if (obj_type == "OCTOMAP")
        {
            i++;
        }
        else if (obj_type == "SPHERE" || obj_type == "BOX" || obj_type == "CYLINDER" || obj_type == "CONE")
        {
            geometric_shapes::constructMarkerFromShape(*(world_obj->object->ptr.solid), mkr);
            mkr.id = world_obj->collision_id;
            mkr.color.g = 1.0;
            mkr.color.a = 1.0;
            marker_array_msg->markers.push_back(mkr);
            i++;
        }
    }

    world_obstacles_pub_->publish(*marker_array_msg);
}

void PathCollisionChecking::convertCollisionModel(const GeometryInformation& geometry_information,
                                                      std::vector<shapes::ShapeMsg>& current_shapes,
                                                      std::vector<geometry_msgs::msg::Pose>& shapes_poses) const
{
    int num_shapes = geometry_information.shapes.size();
    current_shapes.resize(num_shapes);
    shapes_poses.resize(num_shapes);

    for (int i = 0; i < num_shapes; i++)
    {
        shapes::ShapeMsg current_shape;
        shapes::constructMsgFromShape(geometry_information.shapes[i].get(), current_shapes[i]);
        
        shapes_poses[i] = tf2::toMsg(geometry_information.geometry_transforms[i]);
    }
}

void PathCollisionChecking::createRobotCollisionModel(const GeometryInformation& geometry_information, 
                                                       const std::vector<std::string>& additional_link_names)
{
    std::vector<shapes::ShapeMsg> current_shapes;
    std::vector<geometry_msgs::msg::Pose> shapes_poses;
    convertCollisionModel(geometry_information, current_shapes, shapes_poses);

    int num_shapes = geometry_information.shapes.size();
    
    // Clear and rebuild collision geometry cache and corresponding link names
    robot_collision_geometry_.clear();
    collision_link_names_.clear();
    
    // Build mapping from shape index to segment name (chain segments)
    std::vector<std::string> shape_link_names;
    for (int i = 0; i < static_cast<int>(chain_.getNrOfSegments()); ++i)
    {
        KDL::Segment seg = chain_.getSegment(i);
        urdf::CollisionSharedPtr link_coll = model_->links_.at(seg.getName())->collision;
        if (link_coll != nullptr)
        {
            shape_link_names.push_back(seg.getName());
        }
    }
    
    // Append branch link names
    shape_link_names.insert(shape_link_names.end(), additional_link_names.begin(), additional_link_names.end());
    
    for (int i = 0; i < num_shapes; i++)
    {
        Eigen::Affine3d obj_pose;
        tf2::fromMsg(shapes_poses[i], obj_pose);
        if (current_shapes[i].which() == 0)
        {
            robot_collision_checking::FCLObjectPtr obj = std::make_shared<robot_collision_checking::FCLObject>(
                boost::get<shape_msgs::msg::SolidPrimitive>(current_shapes[i]), obj_pose);

            // Create the collision object
            robot_collision_checking::FCLCollisionGeometryPtr cg = robot_collision_checking::fcl_interface::createCollisionGeometry(obj);
            // Cache geometry of robot
            robot_collision_geometry_.push_back(cg);
            // Store corresponding link name
            if (i < static_cast<int>(shape_link_names.size()))
            {
                collision_link_names_.push_back(shape_link_names[i]);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "No link name for geometry %d (total names: %zu)", i, shape_link_names.size());
                collision_link_names_.push_back("unknown_link_" + std::to_string(i));
            }
        }
        else if (current_shapes[i].which() == 1)
        {
            robot_collision_checking::FCLObjectPtr obj = std::make_shared<robot_collision_checking::FCLObject>(
                boost::get<shape_msgs::msg::Mesh>(current_shapes[i]), robot_collision_checking::MESH, obj_pose);

            // Create the collision object
            robot_collision_checking::FCLCollisionGeometryPtr cg = robot_collision_checking::fcl_interface::createCollisionGeometry(obj);
            // Cache geometry of robot
            robot_collision_geometry_.push_back(cg);
            // Store corresponding link name
            if (i < static_cast<int>(shape_link_names.size()))
            {
                collision_link_names_.push_back(shape_link_names[i]);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "No link name for geometry %d (total names: %zu)", i, shape_link_names.size());
                collision_link_names_.push_back("unknown_link_" + std::to_string(i));
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Collision geometry not supported");
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "Built collision model: %zu geometries, %zu link names", 
                robot_collision_geometry_.size(), collision_link_names_.size());
}

std::unique_ptr<shapes::Shape> PathCollisionChecking::constructShape(const urdf::Geometry* geom) const
{
    assert(geom != nullptr);

    std::unique_ptr<shapes::Shape> result = NULL;
    switch (geom->type)
    {
    case urdf::Geometry::SPHERE:
    {
        result = std::unique_ptr<shapes::Shape>(new shapes::Sphere(dynamic_cast<const urdf::Sphere *>(geom)->radius));
        break;
    }
    case urdf::Geometry::BOX:
    {
        urdf::Vector3 dim = dynamic_cast<const urdf::Box *>(geom)->dim;
        result = std::unique_ptr<shapes::Shape>(new shapes::Box(dim.x, dim.y, dim.z));
        break;
    }
    case urdf::Geometry::CYLINDER:
    {
        result = std::unique_ptr<shapes::Shape>(new shapes::Cylinder(dynamic_cast<const urdf::Cylinder *>(geom)->radius,
                                                                     dynamic_cast<const urdf::Cylinder *>(geom)->length));
        break;
    }
    case urdf::Geometry::MESH:
    {
        const urdf::Mesh *mesh = dynamic_cast<const urdf::Mesh *>(geom);
        if (!mesh->filename.empty())
        {
            Eigen::Vector3d scale(mesh->scale.x, mesh->scale.y, mesh->scale.z);
            result = std::unique_ptr<shapes::Shape>(shapes::createMeshFromResource(mesh->filename, scale));
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Empty mesh filename");
        }
        break;
    }
    default:
    {
        RCLCPP_ERROR(this->get_logger(), "Unknown geometry type: %d", (int)geom->type);
        break;
    }
    }
    return (result);
}

void PathCollisionChecking::getKDLKinematicInformation(const KDL::JntArray& kdl_joint_positions, Eigen::Affine3d& T,
                                                           Eigen::Matrix<double, 6, Eigen::Dynamic>& Jac, int segment) const
{
    KDL::Frame cartpos;
    KDL::Jacobian base_J_link_origin;
    base_J_link_origin.resize(ndof_);
    if (segment != -1)
    {
        kdl_fk_solver_->JntToCart(kdl_joint_positions, cartpos, segment);
        kdl_dfk_solver_->JntToJac(kdl_joint_positions, base_J_link_origin, segment);
    }
    else
    {
        kdl_fk_solver_->JntToCart(kdl_joint_positions, cartpos);
        kdl_dfk_solver_->JntToJac(kdl_joint_positions, base_J_link_origin);
    }
    tf2::transformKDLToEigen(cartpos, T);

    Jac = base_J_link_origin.data;
}

void PathCollisionChecking::getCollisionModel(const KDL::JntArray& kdl_joint_positions, GeometryInformation& geometry_information) const
{
    geometry_information.clear();
    Eigen::Affine3d link_origin_T_collision_origin, base_T_link_origin, base_T_collision_origin;

    // Calculates the segment's collision geomtery
    // The transform to the origin of the collision geometry
    // The Jacobian matrix at the origin of the collision geometry
    int num_segments = chain_.getNrOfSegments();
    for (int i = 0; i < num_segments; ++i)
    {
        // Get current segment
        KDL::Segment seg = chain_.getSegment(i);
        // Get collision geometry
        urdf::CollisionSharedPtr link_coll = model_->links_.at(seg.getName())->collision;
        // If collision geometry does not exist at this link of the kinematic chain
        if (link_coll == nullptr)
        {
            // No collision geometry, skip this segment entirely
            // Don't add to any vectors to keep indices consistent
            continue;
        }
        
        // Get collision geometry's shape
        std::unique_ptr<shapes::Shape> shape = constructShape(link_coll->geometry.get());
            // Get collision origin
            Eigen::Vector3d origin_Trans_collision(link_coll->origin.position.x, 
                                                   link_coll->origin.position.y, 
                                                   link_coll->origin.position.z);

            Eigen::Quaterniond origin_Quat_collision(link_coll->origin.rotation.w, 
                                                     link_coll->origin.rotation.x, 
                                                     link_coll->origin.rotation.y, 
                                                     link_coll->origin.rotation.z);

            link_origin_T_collision_origin.translation() = origin_Trans_collision;
            link_origin_T_collision_origin.linear() = origin_Quat_collision.toRotationMatrix();

            // Finds cartesian pose w.r.t to base frame
            Eigen::Matrix<double, 6, Eigen::Dynamic> base_J_collision_origin, base_J_link_origin;
            getKDLKinematicInformation(kdl_joint_positions, base_T_link_origin, base_J_link_origin, i + 1);
            base_T_collision_origin = base_T_link_origin * link_origin_T_collision_origin;
            Eigen::Vector3d base_L_link_collision = (base_T_link_origin.linear() * link_origin_T_collision_origin.translation());
            // Screw transform to collision origin
            screwTransform(base_J_link_origin, base_L_link_collision, base_J_collision_origin);

            // Push back solutions
            geometry_information.shapes.push_back(std::move(shape));
            geometry_information.geometry_transforms.push_back(base_T_collision_origin);
            geometry_information.geometry_jacobians.push_back(base_J_collision_origin);
    }
}

std::vector<std::string> PathCollisionChecking::addBranchCollisionGeometry(const KDL::JntArray& joint_positions, GeometryInformation& geometry_information) const
{
    std::vector<std::string> branch_link_names;  // Track names of added branch links
    
    // Build a set of link names that are in the main kinematic chain
    std::set<std::string> chain_links;
    for (int i = 0; i < static_cast<int>(chain_.getNrOfSegments()); i++)
    {
        chain_links.insert(chain_.getSegment(i).getName());
    }

    // Iterate through all URDF links to find branches with collision geometry
    for (const auto& link_pair : model_->links_)
    {
        const std::string& link_name = link_pair.first;
        const urdf::LinkSharedPtr& link = link_pair.second;

        // Skip if this link is already in the chain
        if (chain_links.find(link_name) != chain_links.end())
            continue;

        // Skip if this link doesn't have collision geometry
        if (!link->collision)
            continue;

        RCLCPP_DEBUG(this->get_logger(), "Processing branch link: %s", link_name.c_str());

        // Compute forward kinematics for this branch link using the tree
        KDL::TreeFkSolverPos_recursive tree_fk_solver(tree_);
        KDL::Frame link_frame;
        
        // The tree FK solver can work with the same joint positions as the chain
        // since branch links are typically fixed joints off chain segments
        int fk_result = tree_fk_solver.JntToCart(joint_positions, link_frame, link_name);
        if (fk_result < 0)
        {
            RCLCPP_WARN(this->get_logger(), "FK failed for branch link: %s", link_name.c_str());
            continue;
        }

        // Get collision geometry shape
        std::unique_ptr<shapes::Shape> shape = constructShape(link->collision->geometry.get());
        if (!shape)
        {
            RCLCPP_WARN(this->get_logger(), "Failed to construct shape for branch link: %s", link_name.c_str());
            continue;
        }

        // Get collision origin transform
        Eigen::Affine3d link_origin_T_collision_origin;
        Eigen::Vector3d origin_trans(link->collision->origin.position.x,
                                     link->collision->origin.position.y,
                                     link->collision->origin.position.z);
        Eigen::Quaterniond origin_quat(link->collision->origin.rotation.w,
                                        link->collision->origin.rotation.x,
                                        link->collision->origin.rotation.y,
                                        link->collision->origin.rotation.z);
        link_origin_T_collision_origin.translation() = origin_trans;
        link_origin_T_collision_origin.linear() = origin_quat.toRotationMatrix();

        // Convert KDL Frame to Eigen transform
        Eigen::Affine3d base_T_link_origin;
        tf2::transformKDLToEigen(link_frame, base_T_link_origin);

        // Compute final transform
        Eigen::Affine3d base_T_collision_origin = base_T_link_origin * link_origin_T_collision_origin;

        // For branch links, we can't easily compute the Jacobian, so we'll use a zero Jacobian
        // This means they won't contribute to polytope calculations, but will be used for collision checking
        Eigen::Matrix<double, 6, Eigen::Dynamic> zero_jacobian;
        zero_jacobian.resize(6, joint_positions.rows());
        zero_jacobian.setZero();

        // Add to geometry information
        geometry_information.shapes.push_back(std::move(shape));
        geometry_information.geometry_transforms.push_back(base_T_collision_origin);
        geometry_information.geometry_jacobians.push_back(zero_jacobian);
        branch_link_names.push_back(link_name);  // Track this link name
        
        RCLCPP_DEBUG(this->get_logger(), "Added branch collision geometry for: %s", link_name.c_str());
    }
    
    return branch_link_names;
}

void PathCollisionChecking::getJacobian(const sensor_msgs::msg::JointState& joint_state, Eigen::Matrix<double, 6, Eigen::Dynamic>& Jac) const
{
    KDL::JntArray kdl_joint_positions(ndof_);
    Eigen::Affine3d base_T_ee;
    jointStatetoKDLJointArray(chain_, joint_state, kdl_joint_positions);
    getKDLKinematicInformation(kdl_joint_positions, base_T_ee, Jac);
}

// void PathCollisionChecking::getTransform(const sensor_msgs::msg::JointState& joint_state, Eigen::Affine3d& T) const
// {
//     KDL::JntArray kdl_joint_positions(ndof_);
//     Eigen::Matrix<double, 6, Eigen::Dynamic> Jac;
//     jointStatetoKDLJointArray(chain_, joint_state, kdl_joint_positions);
//     getKDLKinematicInformation(kdl_joint_positions, T, Jac);
// }

// void PathCollisionChecking::getCartPos(const sensor_msgs::msg::JointState& joint_state, geometry_msgs::msg::Pose& geo_pose) const
// {
//     KDL::JntArray kdl_joint_positions(ndof_);
//     jointStatetoKDLJointArray(chain_, joint_state, kdl_joint_positions);

//     KDL::Frame cartpos;
//     kdl_fk_solver_->JntToCart(kdl_joint_positions, cartpos);

//     Eigen::Affine3d T;
//     tf2::transformKDLToEigen(cartpos, T);
//     geo_pose = tf2::toMsg(T);
// }
} // namespace constrained_manipulability