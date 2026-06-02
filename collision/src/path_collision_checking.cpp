#include <cassert>
#include <chrono>

#include <rclcpp/logging.hpp>

#include <geometric_shapes/shape_to_marker.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <shape_msgs/msg/mesh.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "arm_control/msg/matrix.hpp"

#include "collision/path_collision_checking.hpp"

#include <set>
#include <cmath>
#include <limits>
#include <Eigen/Dense>
#include <geometry_msgs/msg/pose.hpp>

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace constrained_manipulability
{
namespace
{
constexpr double kSlowCollisionStateWarnMs = 100.0;
constexpr double kSlowCollisionBatchWarnMs = 500.0;

struct SimplifiedBoxTimingStats
{
    uint64_t call_count = 0;
    uint64_t mesh_call_count = 0;
    uint64_t total_ns = 0;
    uint64_t mesh_extract_ns = 0;
    double last_size_x = 0.0;
    double last_size_y = 0.0;
    double last_size_z = 0.0;
    double last_offset_x = 0.0;
    double last_offset_y = 0.0;
    double last_offset_z = 0.0;
    std::string last_link_name;

    void reset()
    {
        call_count = 0;
        mesh_call_count = 0;
        total_ns = 0;
        mesh_extract_ns = 0;
        last_size_x = 0.0;
        last_size_y = 0.0;
        last_size_z = 0.0;
        last_offset_x = 0.0;
        last_offset_y = 0.0;
        last_offset_z = 0.0;
        last_link_name.clear();
    }
};

thread_local SimplifiedBoxTimingStats g_simplified_box_timing_stats;

struct SelfCollisionBroadphaseStats
{
    uint64_t candidate_pairs = 0;
    uint64_t aabb_rejected_pairs = 0;
    uint64_t narrowphase_pairs = 0;

    void reset()
    {
        candidate_pairs = 0;
        aabb_rejected_pairs = 0;
        narrowphase_pairs = 0;
    }
};

thread_local SelfCollisionBroadphaseStats g_self_collision_broadphase_stats;

inline uint64_t elapsedNanoseconds(const std::chrono::steady_clock::time_point& started_at)
{
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now() - started_at).count());
}

inline double nanosecondsToMilliseconds(const uint64_t elapsed_ns)
{
    return static_cast<double>(elapsed_ns) / 1.0e6;
}

inline bool endsWith(const std::string& value, const std::string& suffix)
{
    return value.size() >= suffix.size() &&
           value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
}

std::unique_ptr<octomap::OcTree> buildDilatedOctomap(
    const octomap::OcTree& source_tree,
    const double dilation_radius)
{
    auto dilated_tree = std::make_unique<octomap::OcTree>(source_tree.getResolution());
    const double resolution = source_tree.getResolution();
    const int max_offset_steps = static_cast<int>(std::ceil(dilation_radius / resolution));
    const double dilation_radius_sq = dilation_radius * dilation_radius;

    for (auto it = source_tree.begin_leafs(), end = source_tree.end_leafs(); it != end; ++it)
    {
        if (!source_tree.isNodeOccupied(*it))
        {
            continue;
        }

        for (int dx = -max_offset_steps; dx <= max_offset_steps; ++dx)
        {
            const double offset_x = static_cast<double>(dx) * resolution;
            for (int dy = -max_offset_steps; dy <= max_offset_steps; ++dy)
            {
                const double offset_y = static_cast<double>(dy) * resolution;
                for (int dz = -max_offset_steps; dz <= max_offset_steps; ++dz)
                {
                    const double offset_z = static_cast<double>(dz) * resolution;
                    const double offset_distance_sq = offset_x * offset_x + offset_y * offset_y + offset_z * offset_z;
                    if (offset_distance_sq > dilation_radius_sq)
                    {
                        continue;
                    }

                    dilated_tree->updateNode(
                        octomap::point3d(it.getX() + offset_x, it.getY() + offset_y, it.getZ() + offset_z),
                        true);
                }
            }
        }
    }

    dilated_tree->updateInnerOccupancy();
    return dilated_tree;
}

void populateOccupiedVoxelCloud(
    const octomap::OcTree& occupancy_tree,
    const builtin_interfaces::msg::Time& stamp,
    const std::string& target_frame,
    const Eigen::Affine3d& octomap_pose_wrt_base,
    sensor_msgs::msg::PointCloud2& occupied_voxel_cloud)
{
    std::size_t occupied_voxel_count = 0;
    for (auto it = occupancy_tree.begin_leafs(), end = occupancy_tree.end_leafs(); it != end; ++it)
    {
        if (occupancy_tree.isNodeOccupied(*it))
        {
            ++occupied_voxel_count;
        }
    }

    occupied_voxel_cloud.header.stamp = stamp;
    occupied_voxel_cloud.header.frame_id = target_frame;

    sensor_msgs::PointCloud2Modifier modifier(occupied_voxel_cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(occupied_voxel_count);

    sensor_msgs::PointCloud2Iterator<float> iter_x(occupied_voxel_cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(occupied_voxel_cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(occupied_voxel_cloud, "z");

    for (auto it = occupancy_tree.begin_leafs(), end = occupancy_tree.end_leafs(); it != end; ++it)
    {
        if (!occupancy_tree.isNodeOccupied(*it))
        {
            continue;
        }

        const Eigen::Vector3d voxel_center_in_octomap_frame(it.getX(), it.getY(), it.getZ());
        const Eigen::Vector3d voxel_center_in_base_frame = octomap_pose_wrt_base * voxel_center_in_octomap_frame;
        *iter_x = static_cast<float>(voxel_center_in_base_frame.x());
        *iter_y = static_cast<float>(voxel_center_in_base_frame.y());
        *iter_z = static_cast<float>(voxel_center_in_base_frame.z());
        ++iter_x;
        ++iter_y;
        ++iter_z;
    }
}
}  // namespace

PathCollisionChecking::PathCollisionChecking(const rclcpp::NodeOptions& options) : Node("path_collision_checking", options)
{
    // Populate private properties
    base_link_ = this->declare_parameter<std::string>("root", "base_link");
    tip_ = this->declare_parameter<std::string>("tip", "ee_link");
    mode_ = this->declare_parameter<std::string>("mode", "full");
    simplify_mobile_base_collision_geometry_ = this->declare_parameter<bool>("simplify_mobile_base_collision_geometry", true);
    simplified_mobile_base_anchor_link_ = this->declare_parameter<std::string>("simplified_mobile_base_anchor_link", "turret_link");
    simplified_mobile_base_stop_link_ = this->declare_parameter<std::string>("simplified_mobile_base_stop_link", "column_link");
    simplified_mobile_base_box_inflation_ = this->declare_parameter<std::vector<double>>(
        "simplified_mobile_base_box_inflation", std::vector<double>{0.20, 0.0, 0.10});
    simplify_additional_collision_box_geometry_ = this->declare_parameter<bool>("simplify_additional_collision_box_geometry", false);
    simplified_collision_box_link_suffixes_ = this->declare_parameter<std::vector<std::string>>(
        "simplified_collision_box_link_suffixes",
        std::vector<std::string>{"plate_link"});
    publish_debug_octomap_occupied_voxels_ = this->declare_parameter<bool>("publish_debug_octomap_occupied_voxels", true);
    log_detailed_collision_events_ = this->declare_parameter<bool>("log_detailed_collision_events", false);
    this->declare_parameter<double>("octomap_dilation_radius", 0.071);
    simplified_collision_skip_link_suffixes_ = this->declare_parameter<std::vector<std::string>>(
        "simplified_collision_skip_link_suffixes",
        std::vector<std::string>{"sensor_A_link", "sensor_B_link", "sensor_C_link", "ee_cylinder_link"});
    simplified_collision_box_inflation_ = this->declare_parameter<std::vector<double>>(
        "simplified_collision_box_inflation", std::vector<double>{0.0, 0.0, 0.0});
    if (simplified_mobile_base_box_inflation_.size() != 3)
    {
        RCLCPP_WARN(this->get_logger(),
                    "simplified_mobile_base_box_inflation must have 3 values. Using default [0.10, 0.10, 0.20].");
        simplified_mobile_base_box_inflation_ = {0.10, 0.10, 0.20};
    }
    if (simplified_collision_box_inflation_.size() != 3)
    {
        RCLCPP_WARN(this->get_logger(),
                    "simplified_collision_box_inflation must have 3 values. Using default [0.0, 0.0, 0.0].");
        simplified_collision_box_inflation_ = {0.0, 0.0, 0.0};
    }

    // Arm-mode box geometry parameters. Values are given in arm_base frame convention.
    // The collision node converts them to base_link frame internally:
    //   centre_base_link = (-x, -y, z),  rotation_base_link = rotation_arm_base + 180°
    arm_mode_base_raw_box_dims_ = this->declare_parameter<std::vector<double>>(
        "arm_mode_base_raw_box_dims", std::vector<double>{0.8802, 0.6801, 1.0120});
    arm_mode_base_box_center_ = this->declare_parameter<std::vector<double>>(
        "arm_mode_base_box_center", std::vector<double>{-0.2037, 0.2028, -0.6060});
    arm_mode_base_box_rotation_z_deg_ = this->declare_parameter<double>(
        "arm_mode_base_box_rotation_z_deg", -45.0);
    if (arm_mode_base_raw_box_dims_.size() != 3)
    {
        RCLCPP_WARN(this->get_logger(),
                    "arm_mode_base_raw_box_dims must have 3 values. Using default [0.8802, 0.6801, 1.0120].");
        arm_mode_base_raw_box_dims_ = {0.8802, 0.6801, 1.0120};
    }
    if (arm_mode_base_box_center_.size() != 3)
    {
        RCLCPP_WARN(this->get_logger(),
                    "arm_mode_base_box_center must have 3 values. Using default [-0.2037, 0.2028, -0.6060].");
        arm_mode_base_box_center_ = {-0.2037, 0.2028, -0.6060};
    }
    
    // Validate mode parameter
    if (mode_ != "arm" && mode_ != "full")
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid mode '%s'. Must be 'arm' or 'full'. Defaulting to 'full'.", mode_.c_str());
        mode_ = "full";
    }
    
    RCLCPP_INFO(this->get_logger(), "Collision checking mode: %s", mode_.c_str());
    
    // For collision node in /collision namespace, prefix visualization frames
    std::string node_namespace = this->get_namespace();
    if (node_namespace == "/collision" || node_namespace.find("/collision") != std::string::npos)
    {
        visualization_frame_ = "collision/" + base_link_;
    }
    else
    {
        visualization_frame_ = base_link_;
    }

    distance_threshold_ = this->declare_parameter<double>("distance_threshold", 0.3);
    dangerfield_ = this->declare_parameter<double>("dangerfield", 10.0);

    double lin_limit = this->declare_parameter<double>("linearization_limit", 0.1);

    filter_robot_ = this->declare_parameter<bool>("filter_robot", false);
    publish_tested_joint_states_ = this->declare_parameter<bool>("publish_tested_joint_states", false);
    short_circuit_env_on_self_collision_ = this->declare_parameter<bool>("short_circuit_env_on_self_collision", true);
    log_collision_service_metrics_ = this->declare_parameter<bool>("log_collision_service_metrics", true);
    collision_service_metrics_interval_ = this->declare_parameter<int>("collision_service_metrics_interval", 20);

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
    kdl_tree_fk_solver_.reset(new KDL::TreeFkSolverPos_recursive(tree_));

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
            // Check if URDF joint exists
            if (!urdf_joint)
            {
                RCLCPP_ERROR(this->get_logger(), "Joint '%s' not found in URDF model!", kdl_joint.getName().c_str());
                rclcpp::shutdown();
                return;
            }
            
            // Handle joints without limits (common for continuous joints or base joints)
            if (!urdf_joint->limits)
            {
                RCLCPP_WARN(this->get_logger(), "Joint '%s' has no limits defined in URDF, using defaults", kdl_joint.getName().c_str());
                
                // Continuous joint or joint without limits - assign reasonable defaults
                if (urdf_joint->type == urdf::Joint::CONTINUOUS)
                {
                    qmax_[mvable_jnt] = 2.0 * M_PI;
                    qmin_[mvable_jnt] = -2.0 * M_PI;
                }
                else if (urdf_joint->type == urdf::Joint::PRISMATIC)
                {
                    // Default limits for prismatic joints (e.g., column)
                    qmax_[mvable_jnt] = 1.0;  // 1 meter
                    qmin_[mvable_jnt] = 0.0;
                }
                else
                {
                    // Revolute joint without limits
                    qmax_[mvable_jnt] = M_PI;
                    qmin_[mvable_jnt] = -M_PI;
                }
                
                // Default velocity limit
                qdotmax_[mvable_jnt] = 1.0;  // 1 rad/s or m/s
                qdotmin_[mvable_jnt] = -1.0;
            }
            else
            {
                // Normal case: joint has limits defined
                if (urdf_joint->type == urdf::Joint::CONTINUOUS)
                {
                    qmax_[mvable_jnt] = 2.0 * M_PI;
                    qmin_[mvable_jnt] = -2.0 * M_PI;
                }
                else
                {
                    qmax_[mvable_jnt] = urdf_joint->limits->upper;
                    qmin_[mvable_jnt] = urdf_joint->limits->lower;
                }

                qdotmax_[mvable_jnt] = urdf_joint->limits->velocity;
                qdotmin_[mvable_jnt] = -urdf_joint->limits->velocity;
            }
            
            joint_names[mvable_jnt] = kdl_joint.getName();
            mvable_jnt++;
        }
    }

    buildSimplifiedMobileBaseLinkNames();
    buildCachedSimplifiedCollisionBoxes();
    buildSkippedSimplifiedCollisionLinks();
    buildStaticCollisionShapeCache();

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
    check_collision_poses_server_ = this->create_service<arm_control::srv::CheckCollisionPoses>(
        "check_collision_poses", std::bind(&PathCollisionChecking::checkCollisionPosesCallback, this, std::placeholders::_1, std::placeholders::_2));

    mesh_coll_server_st_ = this->create_service<arm_control::srv::AddRemoveCollisionMeshStamped>(
    "add_remove_collision_mesh_stamped",
    std::bind(&PathCollisionChecking::addRemoveMeshStampedCallback, this, std::placeholders::_1, std::placeholders::_2));

    solid_coll_server_st_ = this->create_service<arm_control::srv::AddRemoveCollisionSolidStamped>(
    "add_remove_collision_solid_stamped",
    std::bind(&PathCollisionChecking::addRemoveSolidStampedCallback, this, std::placeholders::_1, std::placeholders::_2));

    update_pos_server_st_ = this->create_service<arm_control::srv::UpdateCollisionPoseStamped>(
    "update_collision_pose_stamped",
    std::bind(&PathCollisionChecking::updateCollisionObjectPoseStampedCallback, this, std::placeholders::_1, std::placeholders::_2));

    // DISABLED: joint_sub_ creates feedback loop - node publishes to /collision/joint_states
    // and would immediately receive its own messages. Visualization is handled by
    // robot_state_publisher subscribing to /collision/joint_states that we publish.
    // rclcpp::SubscriptionOptions joint_sub_options;
    // joint_sub_options.callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    // joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    //     "/collision/joint_states", rclcpp::QoS(1),
    //     std::bind(&PathCollisionChecking::jointStateCallback, this, std::placeholders::_1),
    //     joint_sub_options);
    
    rclcpp::SubscriptionOptions octo_sub_options;
    octo_sub_options.callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions lin_lim_sub_options;
    lin_lim_sub_options.callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
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

    // Subscribe to real robot joint states for one-time startup warm-up.
    // On the first message the collision model is pre-built and visualization is seeded.
    main_tf_prefix_ = this->declare_parameter<std::string>("main_tf_prefix", "arm_");
    real_joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", rclcpp::QoS(1),
        std::bind(&PathCollisionChecking::warmupFromRealJointState, this, std::placeholders::_1));

    // Initialize robot base collision object based on mode
    initializeRobotBaseCollisionObject();

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
    const auto started_at = std::chrono::steady_clock::now();
    bool env_check_skipped = false;

    try 
    {
        // Merge incoming partial joint state with current real robot state
        sensor_msgs::msg::JointState merged_state = mergeJointStates(req->joint_state);
        sensor_msgs::msg::JointState prefixed_state = convertToNamespaceJointState(merged_state);

        bool self_col, env_col;
        res->in_collision = evaluateCollisionState(
            prefixed_state, self_col, env_col, req->publish_visualization, &env_check_skipped);
        
        if (publish_tested_joint_states_ || req->publish_visualization)
        {
            evaluated_joint_pub_->publish(prefixed_state);
        }
    } 
    catch (const std::exception& e) 
    {
        RCLCPP_ERROR(this->get_logger(), "Error in checkCollisionPoseCallback: %s", e.what());
        res->in_collision = true;
    }
    catch (...)
    {
        RCLCPP_ERROR(this->get_logger(), "Unknown error in checkCollisionPoseCallback");
        res->in_collision = true;
    }

    const auto elapsed_ns = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now() - started_at).count());
    if (nanosecondsToMilliseconds(elapsed_ns) >= kSlowCollisionStateWarnMs)
    {
        RCLCPP_WARN(this->get_logger(),
                    "Slow single collision request: total_ms=%.2f publish_visualization=%d in_collision=%d env_skipped=%d",
                    nanosecondsToMilliseconds(elapsed_ns),
                    req->publish_visualization,
                    res->in_collision,
                    env_check_skipped);
    }
    maybeLogCollisionServiceMetrics(elapsed_ns, req->publish_visualization, env_check_skipped);
}

void PathCollisionChecking::checkCollisionPosesCallback(const std::shared_ptr<arm_control::srv::CheckCollisionPoses::Request> req,
                                                        std::shared_ptr<arm_control::srv::CheckCollisionPoses::Response> res)
{
    const auto started_at = std::chrono::steady_clock::now();
    bool requested_visualization = false;
    bool env_check_skipped = false;

    res->in_collision.reserve(req->joint_states.size());
    res->check_succeeded.reserve(req->joint_states.size());

    for (size_t index = 0; index < req->joint_states.size(); ++index)
    {
        const bool visualize_this_state = req->publish_visualization && index == 0;
        bool state_env_check_skipped = false;

        try
        {
            sensor_msgs::msg::JointState merged_state = mergeJointStates(req->joint_states[index]);
            sensor_msgs::msg::JointState prefixed_state = convertToNamespaceJointState(merged_state);

            bool self_col = false;
            bool env_col = false;
            const bool in_collision = evaluateCollisionState(
                prefixed_state, self_col, env_col, visualize_this_state, &state_env_check_skipped);

            res->in_collision.push_back(in_collision);
            res->check_succeeded.push_back(true);

            if (publish_tested_joint_states_ || visualize_this_state)
            {
                evaluated_joint_pub_->publish(prefixed_state);
            }
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error in checkCollisionPosesCallback for state %zu: %s", index, e.what());
            res->in_collision.push_back(true);
            res->check_succeeded.push_back(false);
        }
        catch (...)
        {
            RCLCPP_ERROR(this->get_logger(), "Unknown error in checkCollisionPosesCallback for state %zu", index);
            res->in_collision.push_back(true);
            res->check_succeeded.push_back(false);
        }

        requested_visualization = requested_visualization || visualize_this_state;
        env_check_skipped = env_check_skipped || state_env_check_skipped;
    }

    const auto elapsed_ns = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now() - started_at).count());
    if (nanosecondsToMilliseconds(elapsed_ns) >= kSlowCollisionBatchWarnMs)
    {
        RCLCPP_WARN(this->get_logger(),
                    "Slow batched collision request: states=%zu total_ms=%.2f avg_state_ms=%.2f publish_visualization=%d env_skipped=%d succeeded=%zu",
                    req->joint_states.size(),
                    nanosecondsToMilliseconds(elapsed_ns),
                    req->joint_states.empty() ? 0.0 : nanosecondsToMilliseconds(elapsed_ns) / static_cast<double>(req->joint_states.size()),
                    requested_visualization,
                    env_check_skipped,
                    std::count(res->check_succeeded.begin(), res->check_succeeded.end(), true));
    }
    maybeLogCollisionServiceMetrics(elapsed_ns, requested_visualization, env_check_skipped);
}

sensor_msgs::msg::JointState PathCollisionChecking::mergeJointStates(const sensor_msgs::msg::JointState& partial) const
{
    // Start with current real robot joint state
    joint_state_mutex_.lock();
    sensor_msgs::msg::JointState merged = joint_state_;
    joint_state_mutex_.unlock();
    
    // Build a map of joint name -> index in merged state for fast lookup
    std::map<std::string, size_t> joint_index_map;
    for (size_t i = 0; i < merged.name.size(); ++i)
    {
        joint_index_map[merged.name[i]] = i;
    }
    
    // Override with values from partial joint state
    for (size_t i = 0; i < partial.name.size(); ++i)
    {
        std::string joint_name = partial.name[i];
        
        // Try to find joint as-is first
        auto it = joint_index_map.find(joint_name);
        
        // If not found and doesn't have arm_ prefix, try adding it
        if (it == joint_index_map.end() && joint_name.find("arm_") != 0)
        {
            std::string prefixed_name = "arm_" + joint_name;
            it = joint_index_map.find(prefixed_name);
            if (it != joint_index_map.end())
            {
                RCLCPP_DEBUG(this->get_logger(), "Auto-prefixed joint: %s -> %s", 
                            joint_name.c_str(), prefixed_name.c_str());
                joint_name = prefixed_name;
            }
        }
        
        // Update position if joint found
        if (it != joint_index_map.end())
        {
            size_t idx = it->second;
            if (i < partial.position.size())
            {
                merged.position[idx] = partial.position[i];
                RCLCPP_DEBUG(this->get_logger(), "Updated joint '%s' to position %.3f", 
                            joint_name.c_str(), partial.position[i]);
            }
            if (i < partial.velocity.size() && idx < merged.velocity.size())
            {
                merged.velocity[idx] = partial.velocity[i];
            }
            if (i < partial.effort.size() && idx < merged.effort.size())
            {
                merged.effort[idx] = partial.effort[i];
            }
        }
        else
        {
            RCLCPP_WARN_ONCE(this->get_logger(), "Joint '%s' not found in robot model", joint_name.c_str());
        }
    }
    
    merged.header.stamp = this->now();
    return merged;
}

sensor_msgs::msg::JointState PathCollisionChecking::convertToNamespaceJointState(const sensor_msgs::msg::JointState& input) const
{
    sensor_msgs::msg::JointState visual;
    visual.header.stamp = this->now();
    visual.position = input.position;
    visual.velocity = input.velocity;
    visual.effort = input.effort;
    
    // Both arm and full modes use arm_ prefix in their collision URDF:
    // - arm mode: ur.urdf.xacro is launched with tf_prefix='arm_' → arm_shoulder_pan_joint, etc.
    // - full mode: mobile_manipulator.urdf.xacro has arm_ prefix built-in
    // Real robot always publishes arm_* names, so no conversion needed.
    visual.name = input.name;
    
    return visual;
}

void PathCollisionChecking::warmupCollisionModelFromJointState(const sensor_msgs::msg::JointState& prefixed_joint_state)
{
    bool self_col = false;
    bool env_col = false;

    // Build robot_collision_geometry_ and exclusions; enable visualization to show green meshes
    evaluateCollisionState(prefixed_joint_state, self_col, env_col, true);

    // Seed /collision/joint_states so robot_state_publisher updates TF
    evaluated_joint_pub_->publish(prefixed_joint_state);

    RCLCPP_INFO(this->get_logger(),
                "Startup collision warm-up complete (self_collision=%s, env_collision=%s)",
                self_col ? "true" : "false",
                env_col ? "true" : "false");
}

void PathCollisionChecking::warmupFromRealJointState(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    // Unsubscribe immediately — we only need the first message.
    real_joint_sub_.reset();

    // Store real robot joint state for later merging with service requests
    joint_state_mutex_.lock();
    joint_state_ = *msg;
    joint_state_mutex_.unlock();

    // Convert joint names to match the collision URDF:
    // - arm mode: strip 'arm_' prefix (ur.urdf.xacro uses shoulder_pan_joint, not arm_shoulder_pan_joint)
    // - full mode: keep names as-is (mobile_manipulator.urdf.xacro keeps arm_ prefix)
    sensor_msgs::msg::JointState raw_state;
    raw_state.header = msg->header;
    raw_state.position = msg->position;
    raw_state.velocity = msg->velocity;
    raw_state.effort   = msg->effort;
    raw_state.name = msg->name;

    sensor_msgs::msg::JointState collision_state = convertToNamespaceJointState(raw_state);

    try
    {
        warmupCollisionModelFromJointState(collision_state);
    }
    catch (const std::exception& e)
    {
        RCLCPP_WARN(this->get_logger(),
                    "Startup warm-up failed: %s", e.what());
    }
    catch (...)
    {
        RCLCPP_WARN(this->get_logger(), "Startup warm-up failed with unknown exception");
    }
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
    
    // Also create tree joint array for branch collision geometry
    KDL::JntArray kdl_tree_joint_positions(tree_.getNrOfJoints());
    kdl_tree_joint_positions.data.setZero();
    jointStatetoKDLTreeJointArray(tree_, joint_state_, kdl_tree_joint_positions);
    joint_state_mutex_.unlock();

    GeometryInformation geometry_information;
    getCollisionModel(kdl_joint_positions, geometry_information);
    std::vector<std::string> branch_link_names = addBranchCollisionGeometry(kdl_tree_joint_positions, geometry_information);

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
            shape_stamped.header.frame_id = visualization_frame_;
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
    const double octomap_dilation_radius = this->get_parameter("octomap_dilation_radius").as_double();

    sensor_msgs::msg::PointCloud2 occupied_voxel_cloud;
    octomap_msgs::msg::Octomap effective_octomap_msg;
    const octomap_msgs::msg::Octomap* effective_octomap = msg.get();

    if (publish_debug_octomap_occupied_voxels_ || octomap_dilation_radius > 0.0)
    {
        std::unique_ptr<octomap::AbstractOcTree> octree(octomap_msgs::msgToMap(*msg));
        const auto* occupancy_tree = dynamic_cast<const octomap::OcTree*>(octree.get());
        if (occupancy_tree == nullptr)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                "Received octomap message that could not be converted to octomap::OcTree for debug publishing/dilation.");
        }
        else
        {
            const octomap::OcTree* effective_tree = occupancy_tree;
            std::unique_ptr<octomap::OcTree> dilated_tree;

            if (octomap_dilation_radius > 0.0)
            {
                dilated_tree = buildDilatedOctomap(*occupancy_tree, octomap_dilation_radius);
                if (octomap_msgs::binaryMapToMsg(*dilated_tree, effective_octomap_msg))
                {
                    effective_octomap_msg.header = msg->header;
                    effective_octomap = &effective_octomap_msg;
                    effective_tree = dilated_tree.get();
                }
                else
                {
                    RCLCPP_WARN_THROTTLE(
                        this->get_logger(), *this->get_clock(), 5000,
                        "Failed to serialize dilated octomap; falling back to the original octomap for collision checks.");
                }
            }

            if (publish_debug_octomap_occupied_voxels_)
            {
                populateOccupiedVoxelCloud(
                    *effective_tree,
                    msg->header.stamp,
                    visualization_frame_,
                    octomap_pose_wrt_base,
                    occupied_voxel_cloud);
            }
        }
    }

    boost::mutex::scoped_lock lock(collision_world_mutex_);
    // Remove the old octomap from the world
    collision_world_->removeCollisionObject(OCTOMAP_ID);
    // Update with the new octomap
    robot_collision_checking::FCLObjectPtr octo_obj = std::make_shared<robot_collision_checking::FCLObject>(
        *effective_octomap, robot_collision_checking::OCTOMAP, octomap_pose_wrt_base);
    // Add the filtered octomap to the collision world
    collision_world_->addCollisionObject(octo_obj, OCTOMAP_ID);

    if (publish_debug_octomap_occupied_voxels_)
    {
        occupied_voxels_pub_->publish(occupied_voxel_cloud);
    }
}

/// Private member methods (excluding ROS callbacks)

void PathCollisionChecking::initializeRobotBaseCollisionObject()
{
    const int MOBILE_BASE_ID = 1001;
    
    RCLCPP_INFO(this->get_logger(), "Initializing robot base collision object (mode=%s)", mode_.c_str());
    
    if (mode_ == "arm")
    {
        // Build the oriented box from shared parameters (arm_base frame convention).
        // Convert to base_link frame: negate X and Y of centre, add 180° to rotation.
        const double base_link_rotation_deg = arm_mode_base_box_rotation_z_deg_ + 180.0;
        const double angle_rad = base_link_rotation_deg * M_PI / 180.0;

        Eigen::Affine3d box_transform = Eigen::Affine3d::Identity();
        box_transform.translation() = Eigen::Vector3d(
            -arm_mode_base_box_center_[0],  // negate X  (arm_base → base_link)
            -arm_mode_base_box_center_[1],  // negate Y
             arm_mode_base_box_center_[2]   // keep Z
        );
        Eigen::Matrix3d rotation;
        rotation = Eigen::AngleAxisd(angle_rad, Eigen::Vector3d::UnitZ());
        box_transform.linear() = rotation;

        // Apply shared inflation on top of the raw dimensions.
        shape_msgs::msg::SolidPrimitive box_solid;
        box_solid.type = shape_msgs::msg::SolidPrimitive::BOX;
        box_solid.dimensions.resize(3);
        box_solid.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] =
            arm_mode_base_raw_box_dims_[0] + 2.0 * simplified_mobile_base_box_inflation_[0];
        box_solid.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] =
            arm_mode_base_raw_box_dims_[1] + 2.0 * simplified_mobile_base_box_inflation_[1];
        box_solid.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] =
            arm_mode_base_raw_box_dims_[2] + 2.0 * simplified_mobile_base_box_inflation_[2];

        robot_collision_checking::FCLObjectPtr box_obj =
            std::make_shared<robot_collision_checking::FCLObject>(box_solid, box_transform);

        addCollisionObject(box_obj, MOBILE_BASE_ID);

        RCLCPP_INFO(this->get_logger(),
            "✅ Added oriented robot base box (ID %d): "
            "raw=[%.3f, %.3f, %.3f]m + inflation=[%.3f, %.3f, %.3f]m = [%.3f, %.3f, %.3f]m, "
            "centre=[%.3f, %.3f, %.3f]m (base_link), rotation_z=%.1f° (base_link)",
            MOBILE_BASE_ID,
            arm_mode_base_raw_box_dims_[0], arm_mode_base_raw_box_dims_[1], arm_mode_base_raw_box_dims_[2],
            simplified_mobile_base_box_inflation_[0], simplified_mobile_base_box_inflation_[1], simplified_mobile_base_box_inflation_[2],
            box_solid.dimensions[0], box_solid.dimensions[1], box_solid.dimensions[2],
            box_transform.translation().x(), box_transform.translation().y(), box_transform.translation().z(),
            base_link_rotation_deg);
    }
    else if (mode_ == "full")
    {
        RCLCPP_INFO(this->get_logger(),
                    "Skipping robot base collision-world object in mode='full'; self-collision geometry remains active.");
    }
    
    // Update visualization
    publishWorldObstacles();
}

bool PathCollisionChecking::addCollisionObject(const robot_collision_checking::FCLObjectPtr& obj, int object_id)
{
    return collision_world_->addCollisionObject(obj, object_id);
}

bool PathCollisionChecking::removeCollisionObject(int object_id)
{
    return collision_world_->removeCollisionObject(object_id);
}

bool PathCollisionChecking::evaluateCollisionState(const sensor_msgs::msg::JointState& joint_state,
                                                   bool& self_collision,
                                                   bool& env_collision,
                                                   bool publish_visualization,
                                                   bool* env_check_skipped)
{
    const auto started_at = std::chrono::steady_clock::now();
    uint64_t joint_state_conversion_ns = 0;
    uint64_t chain_geometry_ns = 0;
    uint64_t branch_geometry_ns = 0;
    uint64_t collision_model_rebuild_ns = 0;
    uint64_t self_collision_ns = 0;
    uint64_t visualization_ns = 0;
    uint64_t env_collision_ns = 0;
    uint64_t geometry_reload_ns = 0;
    bool collision_model_created = false;
    bool collision_model_recreated = false;

    if (env_check_skipped != nullptr)
    {
        *env_check_skipped = false;
    }

    g_simplified_box_timing_stats.reset();
    g_self_collision_broadphase_stats.reset();

    const auto joint_state_started_at = std::chrono::steady_clock::now();
    KDL::JntArray kdl_joint_positions(ndof_);
    jointStatetoKDLJointArray(chain_, joint_state, kdl_joint_positions);
    
    // Also create tree joint array for branch collision geometry
    KDL::JntArray kdl_tree_joint_positions(tree_.getNrOfJoints());
    kdl_tree_joint_positions.data.setZero();
    jointStatetoKDLTreeJointArray(tree_, joint_state, kdl_tree_joint_positions);
    joint_state_conversion_ns = elapsedNanoseconds(joint_state_started_at);

    bool need_shape_data = cached_robot_shape_msgs_.empty();

    GeometryInformation geometry_information;
    const auto chain_geometry_started_at = std::chrono::steady_clock::now();
    getCollisionModel(kdl_joint_positions, geometry_information, need_shape_data);
    chain_geometry_ns = elapsedNanoseconds(chain_geometry_started_at);
    const auto branch_geometry_started_at = std::chrono::steady_clock::now();
    std::vector<std::string> branch_link_names = addBranchCollisionGeometry(
        kdl_tree_joint_positions, geometry_information, need_shape_data);
    branch_geometry_ns = elapsedNanoseconds(branch_geometry_started_at);

    if (robot_collision_geometry_.size() == 0)
    {
        const auto collision_model_started_at = std::chrono::steady_clock::now();
        createRobotCollisionModel(geometry_information, branch_link_names);
        collision_model_rebuild_ns += elapsedNanoseconds(collision_model_started_at);
        collision_model_created = true;
        robot_self_collision_objects_.clear();
        robot_env_collision_objects_.clear();
        
        // Build collision exclusions for adjacent and fixed links
        buildCollisionExclusions();
    }
    
    // Validate sizes match
    const size_t num_collision_geometries = geometry_information.geometry_transforms.size();
    if (robot_collision_geometry_.size() != num_collision_geometries)
    {
        if (!need_shape_data)
        {
            const auto geometry_reload_started_at = std::chrono::steady_clock::now();
            geometry_information.clear();
            getCollisionModel(kdl_joint_positions, geometry_information, true);
            branch_link_names = addBranchCollisionGeometry(kdl_tree_joint_positions, geometry_information, true);
            need_shape_data = true;
            geometry_reload_ns = elapsedNanoseconds(geometry_reload_started_at);
        }

        RCLCPP_ERROR(this->get_logger(), 
                     "Size mismatch: robot_collision_geometry_ has %zu elements but geometry_information has %zu transforms. Recreating collision model.",
                     robot_collision_geometry_.size(), geometry_information.geometry_transforms.size());
        robot_collision_geometry_.clear();
        robot_self_collision_objects_.clear();
        robot_env_collision_objects_.clear();
        const auto collision_model_started_at = std::chrono::steady_clock::now();
        createRobotCollisionModel(geometry_information, branch_link_names);
        collision_model_rebuild_ns += elapsedNanoseconds(collision_model_started_at);
        collision_model_recreated = true;
        buildCollisionExclusions();
    }

    boost::mutex::scoped_lock lock(collision_world_mutex_);

    const auto self_collision_started_at = std::chrono::steady_clock::now();
    self_collision = checkSelfCollision(geometry_information);
    self_collision_ns = elapsedNanoseconds(self_collision_started_at);

    // Display collision model if visualization requested
    if (publish_visualization) {
        const auto visualization_started_at = std::chrono::steady_clock::now();
        displayCollisionModel(geometry_information, {0.1, 0.8, 0.1, 0.6});  // Green with transparency
        visualization_ns = elapsedNanoseconds(visualization_started_at);
    }

    if (self_collision && short_circuit_env_on_self_collision_ && !publish_visualization)
    {
        if (env_check_skipped != nullptr)
        {
            *env_check_skipped = true;
        }
        env_collision = false;

        const uint64_t total_elapsed_ns = elapsedNanoseconds(started_at);
        if (nanosecondsToMilliseconds(total_elapsed_ns) >= kSlowCollisionStateWarnMs)
        {
            RCLCPP_WARN(this->get_logger(),
                        "Slow collision state (self short-circuit): total_ms=%.2f joint_ms=%.2f chain_ms=%.2f branch_ms=%.2f reload_ms=%.2f rebuild_ms=%.2f self_ms=%.2f viz_ms=%.2f env_ms=%.2f shapes=%zu box_calls=%llu mesh_box_calls=%llu box_ms=%.2f mesh_box_ms=%.2f box_link=%s box_size=[%.3f, %.3f, %.3f] box_offset=[%.3f, %.3f, %.3f] created=%d recreated=%d publish_visualization=%d",
                        nanosecondsToMilliseconds(total_elapsed_ns),
                        nanosecondsToMilliseconds(joint_state_conversion_ns),
                        nanosecondsToMilliseconds(chain_geometry_ns),
                        nanosecondsToMilliseconds(branch_geometry_ns),
                        nanosecondsToMilliseconds(geometry_reload_ns),
                        nanosecondsToMilliseconds(collision_model_rebuild_ns),
                        nanosecondsToMilliseconds(self_collision_ns),
                        nanosecondsToMilliseconds(visualization_ns),
                        nanosecondsToMilliseconds(env_collision_ns),
                        geometry_information.geometry_transforms.size(),
                        static_cast<unsigned long long>(g_simplified_box_timing_stats.call_count),
                        static_cast<unsigned long long>(g_simplified_box_timing_stats.mesh_call_count),
                        nanosecondsToMilliseconds(g_simplified_box_timing_stats.total_ns),
                        nanosecondsToMilliseconds(g_simplified_box_timing_stats.mesh_extract_ns),
                        g_simplified_box_timing_stats.last_link_name.c_str(),
                        g_simplified_box_timing_stats.last_size_x,
                        g_simplified_box_timing_stats.last_size_y,
                        g_simplified_box_timing_stats.last_size_z,
                        g_simplified_box_timing_stats.last_offset_x,
                        g_simplified_box_timing_stats.last_offset_y,
                        g_simplified_box_timing_stats.last_offset_z,
                        collision_model_created,
                        collision_model_recreated,
                        publish_visualization);
        }
        return true;
    }

    env_collision = false;
    int num_shapes = geometry_information.geometry_transforms.size();
    if (robot_env_collision_objects_.size() != static_cast<size_t>(num_shapes))
    {
        robot_env_collision_objects_.clear();
        robot_env_collision_objects_.reserve(num_shapes);
        for (int i = 0; i < num_shapes; ++i)
        {
            robot_env_collision_objects_.push_back(
                std::make_shared<fcl::CollisionObjectd>(robot_collision_geometry_[i]));
        }
    }

    const auto env_collision_started_at = std::chrono::steady_clock::now();
    for (int i = 0; i < num_shapes; i++)
    {
        std::vector<int> collision_object_ids;

        // In arm mode the base mount links (e.g. arm_base_link_inertia) physically sit inside
        // the mobile base box that was added as an environment object.  Their geometry always
        // touches/overlaps the top face of that box by construction, so checking them produces
        // permanent false positives.  Skip them – they cannot move away from the base.
        if (mode_ == "arm" && endsWith(collision_link_names_[i], "base_link_inertia"))
        {
            continue;
        }

        fcl::Transform3d world_to_fcl;
        robot_collision_checking::fcl_interface::transform2fcl(
            geometry_information.geometry_transforms[i], world_to_fcl);
        robot_env_collision_objects_[i]->setTransform(world_to_fcl);
        robot_env_collision_objects_[i]->computeAABB();

        if (collision_world_->checkCollisionObject(robot_env_collision_objects_[i], collision_object_ids))
        {
            env_collision = true;
            if (log_detailed_collision_events_)
            {
                RCLCPP_WARN(this->get_logger(), "Environment collision detected for link %s with object IDs: %zu objects",
                           collision_link_names_[i].c_str(), collision_object_ids.size());
            }
            break;
        }
    }

    env_collision_ns = elapsedNanoseconds(env_collision_started_at);

    const uint64_t total_elapsed_ns = elapsedNanoseconds(started_at);
    if (nanosecondsToMilliseconds(total_elapsed_ns) >= kSlowCollisionStateWarnMs)
    {
        RCLCPP_WARN(this->get_logger(),
                    "Slow collision state: total_ms=%.2f joint_ms=%.2f chain_ms=%.2f branch_ms=%.2f reload_ms=%.2f rebuild_ms=%.2f self_ms=%.2f viz_ms=%.2f env_ms=%.2f shapes=%d box_calls=%llu mesh_box_calls=%llu box_ms=%.2f mesh_box_ms=%.2f box_link=%s box_size=[%.3f, %.3f, %.3f] box_offset=[%.3f, %.3f, %.3f] self=%d env=%d created=%d recreated=%d publish_visualization=%d",
                    nanosecondsToMilliseconds(total_elapsed_ns),
                    nanosecondsToMilliseconds(joint_state_conversion_ns),
                    nanosecondsToMilliseconds(chain_geometry_ns),
                    nanosecondsToMilliseconds(branch_geometry_ns),
                    nanosecondsToMilliseconds(geometry_reload_ns),
                    nanosecondsToMilliseconds(collision_model_rebuild_ns),
                    nanosecondsToMilliseconds(self_collision_ns),
                    nanosecondsToMilliseconds(visualization_ns),
                    nanosecondsToMilliseconds(env_collision_ns),
                    num_shapes,
                    static_cast<unsigned long long>(g_simplified_box_timing_stats.call_count),
                    static_cast<unsigned long long>(g_simplified_box_timing_stats.mesh_call_count),
                    nanosecondsToMilliseconds(g_simplified_box_timing_stats.total_ns),
                    nanosecondsToMilliseconds(g_simplified_box_timing_stats.mesh_extract_ns),
                    g_simplified_box_timing_stats.last_link_name.c_str(),
                    g_simplified_box_timing_stats.last_size_x,
                    g_simplified_box_timing_stats.last_size_y,
                    g_simplified_box_timing_stats.last_size_z,
                    g_simplified_box_timing_stats.last_offset_x,
                    g_simplified_box_timing_stats.last_offset_y,
                    g_simplified_box_timing_stats.last_offset_z,
                    self_collision,
                    env_collision,
                    collision_model_created,
                    collision_model_recreated,
                    publish_visualization);
    }
    
    return self_collision || env_collision;
}


bool PathCollisionChecking::checkSelfCollision(const GeometryInformation& geometry_information)
{
    bool any_collision_active = false;
    int n = geometry_information.geometry_transforms.size();
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

    if (robot_self_collision_objects_.size() != static_cast<size_t>(n_geom))
    {
        robot_self_collision_objects_.clear();
        robot_self_collision_objects_.reserve(n_geom);
        for (int geom_idx = 0; geom_idx < n_geom; ++geom_idx)
        {
            robot_self_collision_objects_.emplace_back(robot_collision_geometry_[geom_idx]);
        }
    }

    for (int geom_idx = 0; geom_idx < n; ++geom_idx)
    {
        Eigen::Isometry3d pose;
        pose.linear() = geometry_information.geometry_transforms[geom_idx].linear();
        pose.translation() = geometry_information.geometry_transforms[geom_idx].translation();
        robot_self_collision_objects_[geom_idx].setTransform(pose);
        robot_self_collision_objects_[geom_idx].computeAABB();
    }

    const fcl::CollisionRequestd request;

    for (const auto& pair : self_collision_check_pairs_)
    {
        const int i = pair.first;
        const int j = pair.second;
        g_self_collision_broadphase_stats.candidate_pairs++;

        if (i >= n || j >= n)
        {
            RCLCPP_ERROR(this->get_logger(),
                         "Precomputed self-collision pair (%d, %d) is out of bounds for %d transforms. Rebuild the collision exclusions.",
                         i, j, n);
            return false;
        }

        bool collision_detected = false;
        if (robot_self_collision_objects_[i].getAABB().overlap(robot_self_collision_objects_[j].getAABB()))
        {
            g_self_collision_broadphase_stats.narrowphase_pairs++;
            fcl::CollisionResultd result;
            fcl::collide(&robot_self_collision_objects_[i], &robot_self_collision_objects_[j], request, result);
            collision_detected = result.isCollision();
        }
        else
        {
            g_self_collision_broadphase_stats.aabb_rejected_pairs++;
        }

        const auto key = std::make_pair(i, j);
        if (collision_detected != links_collision_states_[key])
        {
            links_collision_states_[key] = collision_detected;
            if (collision_detected)
            {
                // Get link names for better debugging - safe access to pre-built collision_link_names_
                std::string link_i_name = (i < n_names) ? collision_link_names_[i] : "link_" + std::to_string(i);
                std::string link_j_name = (j < n_names) ? collision_link_names_[j] : "link_" + std::to_string(j);

                if (log_detailed_collision_events_)
                {
                    RCLCPP_WARN(this->get_logger(), "Self-collision detected between '%s' (index %d) and '%s' (index %d)", 
                                link_i_name.c_str(), i, link_j_name.c_str(), j);
                }
                return true;
            }
            else
            {
                std::string link_i_name = (i < n_names) ? collision_link_names_[i] : "link_" + std::to_string(i);
                std::string link_j_name = (j < n_names) ? collision_link_names_[j] : "link_" + std::to_string(j);

                if (log_detailed_collision_events_)
                {
                    RCLCPP_INFO(this->get_logger(), "Self-collision cleared between '%s' (index %d) and '%s' (index %d)", 
                                link_i_name.c_str(), i, link_j_name.c_str(), j);
                }
            }
        }

        if (collision_detected)
        {
            any_collision_active = true;
            std::string link_i_name = (i < n_names) ? collision_link_names_[i] : "link_" + std::to_string(i);
            std::string link_j_name = (j < n_names) ? collision_link_names_[j] : "link_" + std::to_string(j);
            if (log_detailed_collision_events_)
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                    "Active collision: '%s' (index %d) <-> '%s' (index %d)",
                    link_i_name.c_str(), i, link_j_name.c_str(), j);
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
    self_collision_check_pairs_.clear();
    links_collision_states_.clear();
    
    // Find which index corresponds to each link name
    std::map<std::string, int> link_name_to_index;
    for (size_t i = 0; i < collision_link_names_.size(); ++i)
    {
        link_name_to_index[collision_link_names_[i]] = i;
    }
    
    // 1. Add adjacent chain links (within 2 hops in the kinematic chain)
    // This catches links separated by non-collision segments (e.g., column_link -> arm_base_link -> arm_base_link_inertia)
    int hop_distance = 2;  // Exclude links within this many segments in the chain
    for (int i = 0; i < static_cast<int>(chain_.getNrOfSegments()); ++i)
    {
        std::string link_i = chain_.getSegment(i).getName();
        auto it_i = link_name_to_index.find(link_i);
        if (it_i == link_name_to_index.end()) continue;  // Skip segments without collision geometry
        
        // Check all segments within hop_distance
        for (int j = i + 1; j <= i + hop_distance && j < static_cast<int>(chain_.getNrOfSegments()); ++j)
        {
            std::string link_j = chain_.getSegment(j).getName();
            auto it_j = link_name_to_index.find(link_j);
            if (it_j == link_name_to_index.end()) continue;  // Skip segments without collision geometry
            
            int idx_i = it_i->second;
            int idx_j = it_j->second;
            collision_exclusions_.insert(std::make_pair(std::min(idx_i, idx_j), std::max(idx_i, idx_j)));
            RCLCPP_INFO(this->get_logger(), "Excluding chain-adjacent links: %s (%d) <-> %s (%d) [%d hops]", 
                        link_i.c_str(), idx_i, link_j.c_str(), idx_j, j - i);
        }
    }
    
    // 2. Add parent-child relationships for all links (especially branches)
    // Build parent map from URDF joints - include ALL joint types since adjacent links
    // in the kinematic tree should always be excluded (they're mechanically connected)
    std::map<std::string, std::string> child_to_parent;
    std::map<std::string, int> child_to_joint_type;
    for (const auto& joint_pair : model_->joints_)
    {
        const urdf::JointSharedPtr& joint = joint_pair.second;
        child_to_parent[joint->child_link_name] = joint->parent_link_name;
        child_to_joint_type[joint->child_link_name] = joint->type;
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
                RCLCPP_INFO(this->get_logger(), "Excluding parent-child: %s (%d) <-> %s (%d)", 
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
                    RCLCPP_INFO(this->get_logger(), "Excluding siblings: %s (%d) <-> %s (%d)", 
                                siblings[i].c_str(), idx_i, siblings[j].c_str(), idx_j);
                }
            }
        }
    }
    
    // 4. Exclude all mobile base links from colliding with each other
    // Find all links up to and including column_link in the kinematic chain
    std::set<std::string> mobile_base_links;
    bool found_column_link = false;
    
    for (int i = 0; i < static_cast<int>(chain_.getNrOfSegments()); ++i)
    {
        std::string link_name = chain_.getSegment(i).getName();
        mobile_base_links.insert(link_name);
        
        if (link_name == "column_link")
        {
            found_column_link = true;
            RCLCPP_INFO(this->get_logger(), "Found column_link at chain segment %d", i);
            break;  // Stop after column_link
        }
    }
    
    // Also add links that are branches off the base/turret (lidars, casters, wheels, etc)
    // These are children/descendants of mobile_base_links but not in the main chain
    // Use iterative approach to catch all descendants (including grandchildren)
    bool added_new_links = true;
    int iteration = 0;
    while (added_new_links && iteration < 10)  // Max 10 iterations to prevent infinite loops
    {
        added_new_links = false;
        iteration++;
        
        for (const auto& pair : child_to_parent)
        {
            const std::string& child = pair.first;
            const std::string& parent = pair.second;
            
            // If parent is a mobile base link and child is not already added
            if (mobile_base_links.find(parent) != mobile_base_links.end() && 
                mobile_base_links.find(child) == mobile_base_links.end())
            {
                // Check if child is NOT in the main chain after column_link
                bool child_is_in_arm_chain = false;
                if (found_column_link)
                {
                    for (int i = 0; i < static_cast<int>(chain_.getNrOfSegments()); ++i)
                    {
                        std::string chain_link = chain_.getSegment(i).getName();
                        if (chain_link == "column_link")
                        {
                            // Check segments after column_link (the arm chain)
                            for (int j = i + 1; j < static_cast<int>(chain_.getNrOfSegments()); ++j)
                            {
                                if (chain_.getSegment(j).getName() == child)
                                {
                                    child_is_in_arm_chain = true;
                                    break;
                                }
                            }
                            break;
                        }
                    }
                }
                
                if (!child_is_in_arm_chain)
                {
                    mobile_base_links.insert(child);
                    added_new_links = true;
                    RCLCPP_DEBUG(this->get_logger(), "Added descendant to mobile base: %s (parent: %s, iteration %d)", 
                                child.c_str(), parent.c_str(), iteration);
                }
            }
        }
    }
    
    if (iteration >= 10)
    {
        RCLCPP_WARN(this->get_logger(), "Mobile base link detection reached max iterations");
    }
    
    RCLCPP_INFO(this->get_logger(), "Identified %zu mobile base links", mobile_base_links.size());
    
    // Convert link names to indices and exclude all pairs
    std::vector<int> base_link_indices;
    for (const auto& base_link : mobile_base_links)
    {
        auto it = link_name_to_index.find(base_link);
        if (it != link_name_to_index.end())
        {
            base_link_indices.push_back(it->second);
            RCLCPP_INFO(this->get_logger(), "Mobile base link: %s (index %d)", 
                        base_link.c_str(), it->second);
        }
    }
    
    // Exclude all pairs of mobile base links
    for (size_t i = 0; i < base_link_indices.size(); ++i)
    {
        for (size_t j = i + 1; j < base_link_indices.size(); ++j)
        {
            int idx_i = base_link_indices[i];
            int idx_j = base_link_indices[j];
            collision_exclusions_.insert(std::make_pair(std::min(idx_i, idx_j), std::max(idx_i, idx_j)));
            
            std::string link_i_name = collision_link_names_[idx_i];
            std::string link_j_name = collision_link_names_[idx_j];
            RCLCPP_INFO(this->get_logger(), "Excluding mobile base pair: %s (%d) <-> %s (%d)", 
                        link_i_name.c_str(), idx_i, link_j_name.c_str(), idx_j);
        }
    }
    
    const int num_collision_links = static_cast<int>(collision_link_names_.size());
    const size_t total_possible_pairs = num_collision_links > 1
        ? static_cast<size_t>(num_collision_links) * static_cast<size_t>(num_collision_links - 1) / 2
        : 0;

    self_collision_check_pairs_.reserve(total_possible_pairs > collision_exclusions_.size()
        ? total_possible_pairs - collision_exclusions_.size()
        : 0);

    for (int i = 0; i < num_collision_links; ++i)
    {
        for (int j = i + 1; j < num_collision_links; ++j)
        {
            const auto pair = std::make_pair(i, j);
            if (collision_exclusions_.find(pair) != collision_exclusions_.end())
            {
                continue;
            }
            self_collision_check_pairs_.push_back(pair);
            links_collision_states_[pair] = false;
        }
    }

    RCLCPP_INFO(this->get_logger(),
                "Built %zu collision exclusion pairs and %zu active self-collision pairs (from %zu total pairs)",
                collision_exclusions_.size(),
                self_collision_check_pairs_.size(),
                total_possible_pairs);
}

void PathCollisionChecking::displayCollisionModel(const GeometryInformation& geometry_information, const Eigen::Vector4d& color)
{
    // Publish robot collision geometry (green mesh)
    auto marker_array_msg = std::make_shared<visualization_msgs::msg::MarkerArray>();

    std::vector<shapes::ShapeMsg> current_shapes;
    std::vector<geometry_msgs::msg::Pose> shapes_poses;
    convertCollisionModel(geometry_information, current_shapes, shapes_poses);

    int num_shapes = current_shapes.size();
    for (int i = 0; i < num_shapes; ++i)
    {
        visualization_msgs::msg::Marker mkr;
        if (current_shapes[i].which() == 0)
        {
            geometric_shapes::constructMarkerFromShape(boost::get<shape_msgs::msg::SolidPrimitive>(current_shapes[i]), mkr);
        }
        else if (current_shapes[i].which() == 1)
        {
            geometric_shapes::constructMarkerFromShape(boost::get<shape_msgs::msg::Mesh>(current_shapes[i]), mkr);
        }
        else
        {
            continue;
        }
        mkr.ns = "collision_body";
        mkr.header.frame_id = visualization_frame_;
        mkr.action = visualization_msgs::msg::Marker::ADD;
        mkr.lifetime = rclcpp::Duration(0, 0);
        mkr.id = i;
        mkr.color.r = color(0);
        mkr.color.g = color(1);
        mkr.color.b = color(2);
        mkr.color.a = color(3);

        mkr.pose = shapes_poses[i];

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
        mkr.header.frame_id = visualization_frame_;
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
    int num_shapes = geometry_information.geometry_transforms.size();
    current_shapes.resize(num_shapes);
    shapes_poses.resize(num_shapes);

    const bool use_dynamic_shapes = geometry_information.shapes.size() == static_cast<size_t>(num_shapes);
    const bool use_cached_shapes = cached_robot_shape_msgs_.size() == static_cast<size_t>(num_shapes);

    for (int i = 0; i < num_shapes; i++)
    {
        if (use_dynamic_shapes)
        {
            shapes::constructMsgFromShape(geometry_information.shapes[i].get(), current_shapes[i]);
        }
        else if (use_cached_shapes)
        {
            current_shapes[i] = cached_robot_shape_msgs_[i];
        }
        else
        {
            current_shapes.clear();
            shapes_poses.clear();
            RCLCPP_ERROR(this->get_logger(), "No cached shape data available for %d collision transforms", num_shapes);
            return;
        }
        
        shapes_poses[i] = tf2::toMsg(geometry_information.geometry_transforms[i]);
    }
}

void PathCollisionChecking::createRobotCollisionModel(const GeometryInformation& geometry_information, 
                                                       const std::vector<std::string>& additional_link_names)
{
    std::vector<shapes::ShapeMsg> current_shapes;
    std::vector<geometry_msgs::msg::Pose> shapes_poses;
    convertCollisionModel(geometry_information, current_shapes, shapes_poses);

    int num_shapes = current_shapes.size();
    
    // Clear and rebuild collision geometry cache and corresponding link names
    robot_collision_geometry_.clear();
    collision_link_names_.clear();
    
    // Build mapping from shape index to segment name (chain segments)
    std::vector<std::string> shape_link_names = cached_robot_shape_link_names_;
    if (shape_link_names.size() != static_cast<size_t>(num_shapes))
    {
        shape_link_names.clear();
        for (int i = 0; i < static_cast<int>(chain_.getNrOfSegments()); ++i)
        {
            KDL::Segment seg = chain_.getSegment(i);
            if (shouldSkipSimplifiedCollisionLink(seg.getName()))
            {
                continue;
            }
            urdf::CollisionSharedPtr link_coll = model_->links_.at(seg.getName())->collision;
            if (link_coll != nullptr)
            {
                shape_link_names.push_back(seg.getName());
            }
        }
        shape_link_names.insert(shape_link_names.end(), additional_link_names.begin(), additional_link_names.end());
    }
    
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

void PathCollisionChecking::buildSimplifiedMobileBaseLinkNames()
{
    simplified_mobile_base_link_names_.clear();
    simplified_mobile_base_box_cached_ = false;
    simplified_mobile_base_box_size_.setZero();
    simplified_mobile_base_box_center_offset_.setZero();

    if (!simplify_mobile_base_collision_geometry_)
    {
        return;
    }

    std::map<std::string, std::string> child_to_parent;
    for (const auto& joint_pair : model_->joints_)
    {
        const urdf::JointSharedPtr& joint = joint_pair.second;
        child_to_parent[joint->child_link_name] = joint->parent_link_name;
    }

    bool found_stop_link = false;
    for (int i = 0; i < static_cast<int>(chain_.getNrOfSegments()); ++i)
    {
        const std::string& link_name = chain_.getSegment(i).getName();
        if (link_name == simplified_mobile_base_stop_link_)
        {
            found_stop_link = true;
            break;
        }
        simplified_mobile_base_link_names_.insert(link_name);
    }

    if (!found_stop_link)
    {
        RCLCPP_WARN(this->get_logger(),
                    "Could not find stop link '%s' for mobile base simplification. Disabling simplification.",
                    simplified_mobile_base_stop_link_.c_str());
        simplified_mobile_base_link_names_.clear();
        simplify_mobile_base_collision_geometry_ = false;
        return;
    }

    bool added_new_links = true;
    int iteration = 0;
    while (added_new_links && iteration < 10)
    {
        added_new_links = false;
        iteration++;

        for (const auto& pair : child_to_parent)
        {
            const std::string& child = pair.first;
            const std::string& parent = pair.second;

            if (simplified_mobile_base_link_names_.find(parent) == simplified_mobile_base_link_names_.end() ||
                simplified_mobile_base_link_names_.find(child) != simplified_mobile_base_link_names_.end())
            {
                continue;
            }

            bool child_is_retained_arm_chain = false;
            bool after_stop_link = false;
            for (int i = 0; i < static_cast<int>(chain_.getNrOfSegments()); ++i)
            {
                const std::string chain_link = chain_.getSegment(i).getName();
                if (chain_link == simplified_mobile_base_stop_link_)
                {
                    after_stop_link = true;
                }
                if (after_stop_link && chain_link == child)
                {
                    child_is_retained_arm_chain = true;
                    break;
                }
            }

            if (!child_is_retained_arm_chain)
            {
                simplified_mobile_base_link_names_.insert(child);
                added_new_links = true;
            }
        }
    }

    if (simplified_mobile_base_link_names_.find(simplified_mobile_base_anchor_link_) == simplified_mobile_base_link_names_.end())
    {
        RCLCPP_WARN(this->get_logger(),
                    "Simplification anchor link '%s' is not part of the simplified mobile base cluster. Disabling simplification.",
                    simplified_mobile_base_anchor_link_.c_str());
        simplified_mobile_base_link_names_.clear();
        simplify_mobile_base_collision_geometry_ = false;
        return;
    }

    RCLCPP_INFO(this->get_logger(),
                "Mobile base simplification enabled: anchor=%s stop_before=%s inflation=[%.3f, %.3f, %.3f] links=%zu",
                simplified_mobile_base_anchor_link_.c_str(),
                simplified_mobile_base_stop_link_.c_str(),
                simplified_mobile_base_box_inflation_[0],
                simplified_mobile_base_box_inflation_[1],
                simplified_mobile_base_box_inflation_[2],
                simplified_mobile_base_link_names_.size());

    auto anchor_link_it = model_->links_.find(simplified_mobile_base_anchor_link_);
    if (anchor_link_it == model_->links_.end() || !anchor_link_it->second || !anchor_link_it->second->collision)
    {
        RCLCPP_WARN(this->get_logger(),
                    "Simplification anchor link '%s' does not have collision geometry. Disabling simplification.",
                    simplified_mobile_base_anchor_link_.c_str());
        simplified_mobile_base_link_names_.clear();
        simplify_mobile_base_collision_geometry_ = false;
        return;
    }

    const urdf::CollisionSharedPtr& anchor_collision = anchor_link_it->second->collision;
    if (!computeCollisionBoxBounds(simplified_mobile_base_anchor_link_,
                                   anchor_collision,
                                   simplified_mobile_base_box_inflation_,
                                   simplified_mobile_base_box_size_,
                                   simplified_mobile_base_box_center_offset_))
    {
        RCLCPP_WARN(this->get_logger(),
                    "Failed to compute cached simplified mobile base box for link %s. Disabling simplification.",
                    simplified_mobile_base_anchor_link_.c_str());
        simplified_mobile_base_link_names_.clear();
        simplify_mobile_base_collision_geometry_ = false;
        return;
    }
    simplified_mobile_base_box_cached_ = true;

    RCLCPP_INFO(this->get_logger(),
                "Cached simplified mobile base box for %s: size=[%.3f, %.3f, %.3f] offset=[%.3f, %.3f, %.3f]",
                simplified_mobile_base_anchor_link_.c_str(),
                simplified_mobile_base_box_size_.x(),
                simplified_mobile_base_box_size_.y(),
                simplified_mobile_base_box_size_.z(),
                simplified_mobile_base_box_center_offset_.x(),
                simplified_mobile_base_box_center_offset_.y(),
                simplified_mobile_base_box_center_offset_.z());
}

void PathCollisionChecking::buildCachedSimplifiedCollisionBoxes()
{
    simplified_collision_box_sizes_.clear();
    simplified_collision_box_center_offsets_.clear();

    if (!simplify_additional_collision_box_geometry_)
    {
        return;
    }

    size_t cached_count = 0;
    for (const auto& link_pair : model_->links_)
    {
        const std::string& link_name = link_pair.first;
        const urdf::LinkSharedPtr& link = link_pair.second;
        if (!link || !link->collision)
        {
            continue;
        }

        bool matches_suffix = false;
        for (const auto& suffix : simplified_collision_box_link_suffixes_)
        {
            if (!suffix.empty() && endsWith(link_name, suffix))
            {
                matches_suffix = true;
                break;
            }
        }

        if (!matches_suffix || shouldUseSimplifiedMobileBaseGeometry(link_name))
        {
            continue;
        }

        Eigen::Vector3d box_size = Eigen::Vector3d::Zero();
        Eigen::Vector3d center_offset = Eigen::Vector3d::Zero();
        bool used_mesh_geometry = false;
        if (!computeCollisionBoxBounds(link_name,
                                       link->collision,
                                       simplified_collision_box_inflation_,
                                       box_size,
                                       center_offset,
                                       &used_mesh_geometry))
        {
            RCLCPP_WARN(this->get_logger(),
                        "Failed to compute cached simplified collision box for link %s",
                        link_name.c_str());
            continue;
        }

        expandCachedSimplifiedCollisionBox(link_name, link->collision, box_size, center_offset);

        simplified_collision_box_sizes_[link_name] = box_size;
        simplified_collision_box_center_offsets_[link_name] = center_offset;
        ++cached_count;

        RCLCPP_INFO(this->get_logger(),
                    "Cached simplified collision box for %s: size=[%.3f, %.3f, %.3f] offset=[%.3f, %.3f, %.3f] source=%s",
                    link_name.c_str(),
                    box_size.x(),
                    box_size.y(),
                    box_size.z(),
                    center_offset.x(),
                    center_offset.y(),
                    center_offset.z(),
                    used_mesh_geometry ? "mesh" : "primitive");
    }

    RCLCPP_INFO(this->get_logger(),
                "Cached %zu additional simplified collision boxes",
                cached_count);
}

void PathCollisionChecking::buildSkippedSimplifiedCollisionLinks()
{
    simplified_collision_skipped_links_.clear();

    for (const auto& link_pair : model_->links_)
    {
        const std::string& link_name = link_pair.first;
        for (const auto& suffix : simplified_collision_skip_link_suffixes_)
        {
            if (!suffix.empty() && endsWith(link_name, suffix))
            {
                // Keep the ultrasonic sensor links ignored even when the plate-box simplification
                // is disabled. The cylinder is only skipped when the expanded plate box is enabled.
                if (!simplify_additional_collision_box_geometry_ && suffix == "ee_cylinder_link")
                {
                    continue;
                }
                simplified_collision_skipped_links_.insert(link_name);
                break;
            }
        }
    }

    RCLCPP_INFO(this->get_logger(),
                "Skipping %zu enclosed simplified collision links",
                simplified_collision_skipped_links_.size());
    for (const auto& link_name : simplified_collision_skipped_links_)
    {
        RCLCPP_INFO(this->get_logger(), "Skipping enclosed collision link: %s", link_name.c_str());
    }
}

bool PathCollisionChecking::shouldSkipSimplifiedMobileBaseLink(const std::string& link_name) const
{
    return simplify_mobile_base_collision_geometry_ &&
           simplified_mobile_base_link_names_.find(link_name) != simplified_mobile_base_link_names_.end() &&
           link_name != simplified_mobile_base_anchor_link_;
}

bool PathCollisionChecking::shouldSkipSimplifiedCollisionLink(const std::string& link_name) const
{
    return shouldSkipSimplifiedMobileBaseLink(link_name) ||
           simplified_collision_skipped_links_.find(link_name) != simplified_collision_skipped_links_.end();
}

bool PathCollisionChecking::shouldUseSimplifiedMobileBaseGeometry(const std::string& link_name) const
{
    return simplify_mobile_base_collision_geometry_ && link_name == simplified_mobile_base_anchor_link_;
}

bool PathCollisionChecking::shouldUseSimplifiedCollisionBoxGeometry(const std::string& link_name) const
{
    return shouldUseSimplifiedMobileBaseGeometry(link_name) ||
           simplified_collision_box_sizes_.find(link_name) != simplified_collision_box_sizes_.end();
}

bool PathCollisionChecking::buildSimplifiedCollisionBox(const std::string& link_name,
                                                        const urdf::CollisionSharedPtr& link_collision,
                                                        std::unique_ptr<shapes::Shape>* simplified_shape,
                                                        Eigen::Vector3d* collision_frame_center_offset) const
{
    if (shouldUseSimplifiedMobileBaseGeometry(link_name))
    {
        return buildSimplifiedMobileBaseBox(link_name, link_collision, simplified_shape, collision_frame_center_offset);
    }

    const auto size_it = simplified_collision_box_sizes_.find(link_name);
    if (size_it == simplified_collision_box_sizes_.end())
    {
        return false;
    }

    const auto offset_it = simplified_collision_box_center_offsets_.find(link_name);
    const Eigen::Vector3d center_offset = offset_it != simplified_collision_box_center_offsets_.end()
        ? offset_it->second
        : Eigen::Vector3d::Zero();

    if (collision_frame_center_offset != nullptr)
    {
        *collision_frame_center_offset = center_offset;
    }
    if (simplified_shape != nullptr)
    {
        simplified_shape->reset(new shapes::Box(size_it->second.x(), size_it->second.y(), size_it->second.z()));
    }

    g_simplified_box_timing_stats.call_count++;
    g_simplified_box_timing_stats.last_link_name = link_name;
    g_simplified_box_timing_stats.last_size_x = size_it->second.x();
    g_simplified_box_timing_stats.last_size_y = size_it->second.y();
    g_simplified_box_timing_stats.last_size_z = size_it->second.z();
    g_simplified_box_timing_stats.last_offset_x = center_offset.x();
    g_simplified_box_timing_stats.last_offset_y = center_offset.y();
    g_simplified_box_timing_stats.last_offset_z = center_offset.z();
    return true;
}

void PathCollisionChecking::expandCachedSimplifiedCollisionBox(const std::string& anchor_link_name,
                                                               const urdf::CollisionSharedPtr& anchor_collision,
                                                               Eigen::Vector3d& box_size,
                                                               Eigen::Vector3d& box_center_offset) const
{
    if (!anchor_collision || simplified_collision_skipped_links_.empty() || !endsWith(anchor_link_name, "plate_link"))
    {
        return;
    }

    KDL::JntArray zero_joint_positions(tree_.getNrOfJoints());
    zero_joint_positions.data.setZero();

    KDL::Frame anchor_kdl_frame;
    if (kdl_tree_fk_solver_->JntToCart(zero_joint_positions, anchor_kdl_frame, anchor_link_name) < 0)
    {
        RCLCPP_WARN(this->get_logger(),
                    "Failed to compute FK while expanding simplified collision box for %s",
                    anchor_link_name.c_str());
        return;
    }

    Eigen::Affine3d base_T_anchor_link;
    tf2::transformKDLToEigen(anchor_kdl_frame, base_T_anchor_link);

    Eigen::Quaterniond anchor_quat(anchor_collision->origin.rotation.w,
                                   anchor_collision->origin.rotation.x,
                                   anchor_collision->origin.rotation.y,
                                   anchor_collision->origin.rotation.z);
    Eigen::Affine3d anchor_link_T_collision = Eigen::Affine3d::Identity();
    anchor_link_T_collision.linear() = anchor_quat.toRotationMatrix();
    anchor_link_T_collision.translation() = Eigen::Vector3d(anchor_collision->origin.position.x,
                                                            anchor_collision->origin.position.y,
                                                            anchor_collision->origin.position.z);
    const Eigen::Affine3d base_T_anchor_collision = base_T_anchor_link * anchor_link_T_collision;

    Eigen::Vector3d min_corner = box_center_offset - 0.5 * box_size;
    Eigen::Vector3d max_corner = box_center_offset + 0.5 * box_size;

    const std::vector<double> zero_inflation{0.0, 0.0, 0.0};
    size_t expanded_links = 0;
    for (const auto& skipped_link_name : simplified_collision_skipped_links_)
    {
        if (skipped_link_name == anchor_link_name)
        {
            continue;
        }

        const auto link_it = model_->links_.find(skipped_link_name);
        if (link_it == model_->links_.end() || !link_it->second || !link_it->second->collision)
        {
            continue;
        }

        Eigen::Vector3d skipped_box_size = Eigen::Vector3d::Zero();
        Eigen::Vector3d skipped_box_center_offset = Eigen::Vector3d::Zero();
        if (!computeCollisionBoxBounds(skipped_link_name,
                                       link_it->second->collision,
                                       zero_inflation,
                                       skipped_box_size,
                                       skipped_box_center_offset))
        {
            continue;
        }

        KDL::Frame skipped_kdl_frame;
        if (kdl_tree_fk_solver_->JntToCart(zero_joint_positions, skipped_kdl_frame, skipped_link_name) < 0)
        {
            RCLCPP_WARN(this->get_logger(),
                        "Failed to compute FK for enclosed skipped link %s while expanding %s",
                        skipped_link_name.c_str(),
                        anchor_link_name.c_str());
            continue;
        }

        Eigen::Affine3d base_T_skipped_link;
        tf2::transformKDLToEigen(skipped_kdl_frame, base_T_skipped_link);

        const urdf::CollisionSharedPtr& skipped_collision = link_it->second->collision;
        Eigen::Quaterniond skipped_quat(skipped_collision->origin.rotation.w,
                                        skipped_collision->origin.rotation.x,
                                        skipped_collision->origin.rotation.y,
                                        skipped_collision->origin.rotation.z);
        Eigen::Affine3d skipped_link_T_collision = Eigen::Affine3d::Identity();
        skipped_link_T_collision.linear() = skipped_quat.toRotationMatrix();
        skipped_link_T_collision.translation() = Eigen::Vector3d(skipped_collision->origin.position.x,
                                                                 skipped_collision->origin.position.y,
                                                                 skipped_collision->origin.position.z);
        const Eigen::Affine3d anchor_collision_T_skipped_collision =
            base_T_anchor_collision.inverse() * (base_T_skipped_link * skipped_link_T_collision);

        for (int corner_idx = 0; corner_idx < 8; ++corner_idx)
        {
            Eigen::Vector3d local_corner(
                (corner_idx & 1) ? 0.5 * skipped_box_size.x() : -0.5 * skipped_box_size.x(),
                (corner_idx & 2) ? 0.5 * skipped_box_size.y() : -0.5 * skipped_box_size.y(),
                (corner_idx & 4) ? 0.5 * skipped_box_size.z() : -0.5 * skipped_box_size.z());
            local_corner += skipped_box_center_offset;

            const Eigen::Vector3d anchor_corner = anchor_collision_T_skipped_collision * local_corner;
            min_corner = min_corner.cwiseMin(anchor_corner);
            max_corner = max_corner.cwiseMax(anchor_corner);
        }

        ++expanded_links;
    }

    if (expanded_links == 0)
    {
        return;
    }

    box_size = (max_corner - min_corner).cwiseMax(Eigen::Vector3d::Constant(1e-3));
    box_center_offset = 0.5 * (min_corner + max_corner);

    RCLCPP_INFO(this->get_logger(),
                "Expanded simplified collision box for %s to enclose %zu skipped links: size=[%.3f, %.3f, %.3f] offset=[%.3f, %.3f, %.3f]",
                anchor_link_name.c_str(),
                expanded_links,
                box_size.x(),
                box_size.y(),
                box_size.z(),
                box_center_offset.x(),
                box_center_offset.y(),
                box_center_offset.z());
}

bool PathCollisionChecking::computeCollisionBoxBounds(const std::string& link_name,
                                                      const urdf::CollisionSharedPtr& link_collision,
                                                      const std::vector<double>& inflation,
                                                      Eigen::Vector3d& box_size,
                                                      Eigen::Vector3d& box_center_offset,
                                                      bool* used_mesh_geometry) const
{
    if (!link_collision || !link_collision->geometry)
    {
        return false;
    }

    box_size = Eigen::Vector3d::Zero();
    box_center_offset = Eigen::Vector3d::Zero();
    if (used_mesh_geometry != nullptr)
    {
        *used_mesh_geometry = false;
    }

    double size_x = 0.0;
    double size_y = 0.0;
    double size_z = 0.0;

    switch (link_collision->geometry->type)
    {
    case urdf::Geometry::BOX:
    {
        urdf::Vector3 dim = dynamic_cast<const urdf::Box*>(link_collision->geometry.get())->dim;
        size_x = dim.x;
        size_y = dim.y;
        size_z = dim.z;
        break;
    }
    case urdf::Geometry::CYLINDER:
    {
        const auto* cylinder = dynamic_cast<const urdf::Cylinder*>(link_collision->geometry.get());
        size_x = 2.0 * cylinder->radius;
        size_y = 2.0 * cylinder->radius;
        size_z = cylinder->length;
        break;
    }
    case urdf::Geometry::SPHERE:
    {
        const auto* sphere = dynamic_cast<const urdf::Sphere*>(link_collision->geometry.get());
        size_x = 2.0 * sphere->radius;
        size_y = 2.0 * sphere->radius;
        size_z = 2.0 * sphere->radius;
        break;
    }
    case urdf::Geometry::MESH:
    {
        std::unique_ptr<shapes::Shape> mesh_shape = constructShape(link_collision->geometry.get());
        if (!mesh_shape)
        {
            RCLCPP_WARN(this->get_logger(), "Failed to compute box bounds for mesh link %s", link_name.c_str());
            return false;
        }

        shapes::ShapeMsg shape_msg;
        if (!shapes::constructMsgFromShape(mesh_shape.get(), shape_msg) || shape_msg.which() != 1)
        {
            RCLCPP_WARN(this->get_logger(), "Failed to convert mesh link %s into a mesh message for simplification", link_name.c_str());
            return false;
        }

        const shape_msgs::msg::Mesh mesh_msg = boost::get<shape_msgs::msg::Mesh>(shape_msg);
        if (mesh_msg.vertices.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Mesh link %s has no vertices for simplification", link_name.c_str());
            return false;
        }

        double min_x = std::numeric_limits<double>::infinity();
        double min_y = std::numeric_limits<double>::infinity();
        double min_z = std::numeric_limits<double>::infinity();
        double max_x = -std::numeric_limits<double>::infinity();
        double max_y = -std::numeric_limits<double>::infinity();
        double max_z = -std::numeric_limits<double>::infinity();

        for (const auto& vertex : mesh_msg.vertices)
        {
            min_x = std::min(min_x, static_cast<double>(vertex.x));
            min_y = std::min(min_y, static_cast<double>(vertex.y));
            min_z = std::min(min_z, static_cast<double>(vertex.z));
            max_x = std::max(max_x, static_cast<double>(vertex.x));
            max_y = std::max(max_y, static_cast<double>(vertex.y));
            max_z = std::max(max_z, static_cast<double>(vertex.z));
        }

        size_x = max_x - min_x;
        size_y = max_y - min_y;
        size_z = max_z - min_z;
        box_center_offset = Eigen::Vector3d(
            0.5 * (min_x + max_x),
            0.5 * (min_y + max_y),
            0.5 * (min_z + max_z));
        if (used_mesh_geometry != nullptr)
        {
            *used_mesh_geometry = true;
        }
        break;
    }
    default:
        RCLCPP_WARN(this->get_logger(), "Unsupported geometry type %d for simplified link %s",
                    link_collision->geometry->type, link_name.c_str());
        return false;
    }

    box_size = Eigen::Vector3d(
        std::max(1e-3, size_x + inflation[0]),
        std::max(1e-3, size_y + inflation[1]),
        std::max(1e-3, size_z + inflation[2]));
    return true;
}

bool PathCollisionChecking::buildSimplifiedMobileBaseBox(const std::string& link_name,
                                                         const urdf::CollisionSharedPtr& link_collision,
                                                         std::unique_ptr<shapes::Shape>* simplified_shape,
                                                         Eigen::Vector3d* collision_frame_center_offset) const
{
    const auto started_at = std::chrono::steady_clock::now();
    if (shouldUseSimplifiedMobileBaseGeometry(link_name) && simplified_mobile_base_box_cached_)
    {
        if (collision_frame_center_offset != nullptr)
        {
            *collision_frame_center_offset = simplified_mobile_base_box_center_offset_;
        }
        if (simplified_shape != nullptr)
        {
            simplified_shape->reset(new shapes::Box(
                simplified_mobile_base_box_size_.x(),
                simplified_mobile_base_box_size_.y(),
                simplified_mobile_base_box_size_.z()));
        }

        g_simplified_box_timing_stats.call_count++;
        g_simplified_box_timing_stats.total_ns += elapsedNanoseconds(started_at);
        g_simplified_box_timing_stats.last_link_name = link_name;
        g_simplified_box_timing_stats.last_size_x = simplified_mobile_base_box_size_.x();
        g_simplified_box_timing_stats.last_size_y = simplified_mobile_base_box_size_.y();
        g_simplified_box_timing_stats.last_size_z = simplified_mobile_base_box_size_.z();
        g_simplified_box_timing_stats.last_offset_x = simplified_mobile_base_box_center_offset_.x();
        g_simplified_box_timing_stats.last_offset_y = simplified_mobile_base_box_center_offset_.y();
        g_simplified_box_timing_stats.last_offset_z = simplified_mobile_base_box_center_offset_.z();
        return true;
    }

    Eigen::Vector3d box_size = Eigen::Vector3d::Zero();
    Eigen::Vector3d center_offset = Eigen::Vector3d::Zero();
    bool used_mesh_geometry = false;
    if (!computeCollisionBoxBounds(link_name,
                                   link_collision,
                                   simplified_mobile_base_box_inflation_,
                                   box_size,
                                   center_offset,
                                   &used_mesh_geometry))
    {
        return false;
    }

    if (simplified_shape != nullptr)
    {
        simplified_shape->reset(new shapes::Box(box_size.x(), box_size.y(), box_size.z()));
    }
    if (collision_frame_center_offset != nullptr)
    {
        *collision_frame_center_offset = center_offset;
    }
    if (used_mesh_geometry)
    {
        g_simplified_box_timing_stats.mesh_call_count++;
    }

    g_simplified_box_timing_stats.call_count++;
    g_simplified_box_timing_stats.total_ns += elapsedNanoseconds(started_at);
    g_simplified_box_timing_stats.last_link_name = link_name;
    g_simplified_box_timing_stats.last_size_x = box_size.x();
    g_simplified_box_timing_stats.last_size_y = box_size.y();
    g_simplified_box_timing_stats.last_size_z = box_size.z();
    g_simplified_box_timing_stats.last_offset_x = center_offset.x();
    g_simplified_box_timing_stats.last_offset_y = center_offset.y();
    g_simplified_box_timing_stats.last_offset_z = center_offset.z();

    return true;
}

void PathCollisionChecking::buildStaticCollisionShapeCache()
{
    cached_robot_shape_msgs_.clear();
    cached_robot_shape_link_names_.clear();

    auto append_cached_shape = [this](const std::string& link_name, const urdf::CollisionSharedPtr& link_collision,
                                      std::vector<shapes::ShapeMsg>& shape_msgs,
                                      std::vector<std::string>& link_names) {
        std::unique_ptr<shapes::Shape> shape;
        if (shouldUseSimplifiedCollisionBoxGeometry(link_name))
        {
            if (!buildSimplifiedCollisionBox(link_name, link_collision, &shape, nullptr))
            {
                shape = constructShape(link_collision->geometry.get());
            }
        }
        else
        {
            shape = constructShape(link_collision->geometry.get());
        }
        if (!shape)
        {
            RCLCPP_WARN(this->get_logger(), "Failed to build cached shape for link: %s", link_name.c_str());
            return;
        }

        shapes::ShapeMsg shape_msg;
        if (!shapes::constructMsgFromShape(shape.get(), shape_msg))
        {
            RCLCPP_WARN(this->get_logger(), "Failed to convert cached shape for link: %s", link_name.c_str());
            return;
        }

        shape_msgs.push_back(shape_msg);
        link_names.push_back(link_name);
    };

    std::set<std::string> chain_links;
    for (int i = 0; i < static_cast<int>(chain_.getNrOfSegments()); ++i)
    {
        const std::string& link_name = chain_.getSegment(i).getName();
        if (shouldSkipSimplifiedCollisionLink(link_name))
        {
            continue;
        }
        chain_links.insert(link_name);

        urdf::CollisionSharedPtr link_collision = model_->links_.at(link_name)->collision;
        if (link_collision != nullptr)
        {
            append_cached_shape(link_name, link_collision, cached_robot_shape_msgs_, cached_robot_shape_link_names_);
        }
    }

    for (const auto& link_pair : model_->links_)
    {
        const std::string& link_name = link_pair.first;
        const urdf::LinkSharedPtr& link = link_pair.second;

        if (chain_links.find(link_name) != chain_links.end() || !link->collision)
        {
            continue;
        }

        if (shouldSkipSimplifiedCollisionLink(link_name))
        {
            continue;
        }

        append_cached_shape(link_name, link->collision, cached_robot_shape_msgs_, cached_robot_shape_link_names_);
    }

    RCLCPP_INFO(this->get_logger(), "Built static collision shape cache: %zu shapes", cached_robot_shape_msgs_.size());
}

void PathCollisionChecking::maybeLogCollisionServiceMetrics(uint64_t elapsed_ns, bool requested_visualization,
                                                            bool env_check_skipped)
{
    if (!log_collision_service_metrics_)
    {
        return;
    }

    const uint64_t call_count = collision_service_call_count_.fetch_add(1) + 1;
    collision_service_total_ns_.fetch_add(elapsed_ns);
    self_collision_candidate_pair_total_.fetch_add(g_self_collision_broadphase_stats.candidate_pairs);
    self_collision_aabb_rejected_pair_total_.fetch_add(g_self_collision_broadphase_stats.aabb_rejected_pairs);
    self_collision_narrowphase_pair_total_.fetch_add(g_self_collision_broadphase_stats.narrowphase_pairs);
    if (requested_visualization)
    {
        collision_service_visualized_calls_.fetch_add(1);
    }
    if (env_check_skipped)
    {
        collision_service_short_circuit_count_.fetch_add(1);
    }

    if (collision_service_metrics_interval_ > 0 && (call_count % static_cast<uint64_t>(collision_service_metrics_interval_) == 0))
    {
        const double avg_ms = static_cast<double>(collision_service_total_ns_.load()) /
                              static_cast<double>(call_count) / 1.0e6;
        const uint64_t candidate_pair_total = self_collision_candidate_pair_total_.load();
        const uint64_t aabb_rejected_pair_total = self_collision_aabb_rejected_pair_total_.load();
        const uint64_t narrowphase_pair_total = self_collision_narrowphase_pair_total_.load();
        const double candidate_pairs_avg = static_cast<double>(candidate_pair_total) / static_cast<double>(call_count);
        const double aabb_rejected_pairs_avg = static_cast<double>(aabb_rejected_pair_total) / static_cast<double>(call_count);
        const double narrowphase_pairs_avg = static_cast<double>(narrowphase_pair_total) / static_cast<double>(call_count);
        const double aabb_reject_rate = candidate_pair_total > 0
            ? (static_cast<double>(aabb_rejected_pair_total) / static_cast<double>(candidate_pair_total)) * 100.0
            : 0.0;
        RCLCPP_WARN(this->get_logger(),
                    "Collision service metrics: calls=%llu avg_ms=%.2f visualized=%llu self_short_circuits=%llu self_pairs_avg=%.1f aabb_reject_avg=%.1f narrowphase_avg=%.1f aabb_reject_rate=%.1f%% last_pairs=%llu last_reject=%llu last_narrow=%llu last_ms=%.2f",
                    static_cast<unsigned long long>(call_count),
                    avg_ms,
                    static_cast<unsigned long long>(collision_service_visualized_calls_.load()),
                    static_cast<unsigned long long>(collision_service_short_circuit_count_.load()),
                    candidate_pairs_avg,
                    aabb_rejected_pairs_avg,
                    narrowphase_pairs_avg,
                    aabb_reject_rate,
                    static_cast<unsigned long long>(g_self_collision_broadphase_stats.candidate_pairs),
                    static_cast<unsigned long long>(g_self_collision_broadphase_stats.aabb_rejected_pairs),
                    static_cast<unsigned long long>(g_self_collision_broadphase_stats.narrowphase_pairs),
                    static_cast<double>(elapsed_ns) / 1.0e6);
    }
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

void PathCollisionChecking::getCollisionModel(const KDL::JntArray& kdl_joint_positions,
                                              GeometryInformation& geometry_information,
                                              bool populate_shapes) const
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
        if (shouldSkipSimplifiedCollisionLink(seg.getName()))
        {
            continue;
        }
        // Get collision geometry
        urdf::CollisionSharedPtr link_coll = model_->links_.at(seg.getName())->collision;
        // If collision geometry does not exist at this link of the kinematic chain
        if (link_coll == nullptr)
        {
            // No collision geometry, skip this segment entirely
            // Don't add to any vectors to keep indices consistent
            continue;
        }
        
        std::unique_ptr<shapes::Shape> shape;
        Eigen::Vector3d collision_frame_center_offset = Eigen::Vector3d::Zero();
        if (populate_shapes)
        {
            if (shouldUseSimplifiedCollisionBoxGeometry(seg.getName()))
            {
                if (!buildSimplifiedCollisionBox(seg.getName(), link_coll, &shape, &collision_frame_center_offset))
                {
                    shape = constructShape(link_coll->geometry.get());
                }
            }
            else
            {
                shape = constructShape(link_coll->geometry.get());
            }
        }
        else if (shouldUseSimplifiedCollisionBoxGeometry(seg.getName()))
        {
            buildSimplifiedCollisionBox(seg.getName(), link_coll, nullptr, &collision_frame_center_offset);
        }

        // Get collision origin
        Eigen::Quaterniond origin_Quat_collision(link_coll->origin.rotation.w, 
                                                 link_coll->origin.rotation.x, 
                                                 link_coll->origin.rotation.y, 
                                                 link_coll->origin.rotation.z);
        Eigen::Matrix3d origin_rotation = origin_Quat_collision.toRotationMatrix();

        Eigen::Vector3d origin_Trans_collision(link_coll->origin.position.x, 
                                               link_coll->origin.position.y, 
                                               link_coll->origin.position.z);
        origin_Trans_collision += origin_rotation * collision_frame_center_offset;

        link_origin_T_collision_origin.translation() = origin_Trans_collision;
        link_origin_T_collision_origin.linear() = origin_rotation;

        // Finds cartesian pose w.r.t to base frame
        Eigen::Matrix<double, 6, Eigen::Dynamic> base_J_collision_origin, base_J_link_origin;
        getKDLKinematicInformation(kdl_joint_positions, base_T_link_origin, base_J_link_origin, i + 1);
        base_T_collision_origin = base_T_link_origin * link_origin_T_collision_origin;
        Eigen::Vector3d base_L_link_collision = (base_T_link_origin.linear() * link_origin_T_collision_origin.translation());
        // Screw transform to collision origin
        screwTransform(base_J_link_origin, base_L_link_collision, base_J_collision_origin);

        // Push back solutions
        if (populate_shapes)
        {
            geometry_information.shapes.push_back(std::move(shape));
        }
        geometry_information.geometry_transforms.push_back(base_T_collision_origin);
        geometry_information.geometry_jacobians.push_back(base_J_collision_origin);
    }
}

std::vector<std::string> PathCollisionChecking::addBranchCollisionGeometry(const KDL::JntArray& joint_positions,
                                                                           GeometryInformation& geometry_information,
                                                                           bool populate_shapes) const
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

        if (shouldSkipSimplifiedCollisionLink(link_name))
            continue;

        RCLCPP_DEBUG(this->get_logger(), "Processing branch link: %s", link_name.c_str());

        KDL::Frame link_frame;
        
        // The tree FK solver can work with the same joint positions as the chain
        // since branch links are typically fixed joints off chain segments
        int fk_result = kdl_tree_fk_solver_->JntToCart(joint_positions, link_frame, link_name);
        if (fk_result < 0)
        {
            RCLCPP_WARN(this->get_logger(), "FK failed for branch link: %s", link_name.c_str());
            continue;
        }

        std::unique_ptr<shapes::Shape> shape;
        Eigen::Vector3d collision_frame_center_offset = Eigen::Vector3d::Zero();
        if (populate_shapes)
        {
            if (shouldUseSimplifiedCollisionBoxGeometry(link_name))
            {
                if (!buildSimplifiedCollisionBox(link_name, link->collision, &shape, &collision_frame_center_offset))
                {
                    shape = constructShape(link->collision->geometry.get());
                }
            }
            else
            {
                // Get collision geometry shape
                shape = constructShape(link->collision->geometry.get());
            }
            if (!shape)
            {
                RCLCPP_WARN(this->get_logger(), "Failed to construct shape for branch link: %s", link_name.c_str());
                continue;
            }
        }
        else if (shouldUseSimplifiedCollisionBoxGeometry(link_name))
        {
            buildSimplifiedCollisionBox(link_name, link->collision, nullptr, &collision_frame_center_offset);
        }

        // Get collision origin transform
        Eigen::Vector3d origin_trans(link->collision->origin.position.x,
                                     link->collision->origin.position.y,
                                     link->collision->origin.position.z);
        Eigen::Quaterniond origin_quat(link->collision->origin.rotation.w,
                                        link->collision->origin.rotation.x,
                                        link->collision->origin.rotation.y,
                                        link->collision->origin.rotation.z);
        origin_trans += origin_quat.toRotationMatrix() * collision_frame_center_offset;
        Eigen::Affine3d link_origin_T_collision_origin;
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
        if (populate_shapes)
        {
            geometry_information.shapes.push_back(std::move(shape));
        }
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

} // namespace constrained_manipulability
