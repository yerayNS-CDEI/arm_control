#include <cassert>
#include <chrono>

#include <rclcpp/logging.hpp>

#include <geometric_shapes/shape_to_marker.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <shape_msgs/msg/mesh.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include "arm_control/msg/matrix.hpp"

#include "collision/constrained_manipulability_mod.hpp"

#include <set>
#include <cmath>
#include <Eigen/Dense>
#include <geometry_msgs/msg/pose.hpp>


namespace constrained_manipulability
{
ConstrainedManipulabilityMod::ConstrainedManipulabilityMod(const rclcpp::NodeOptions& options) : Node("constrained_manipulability_mod", options)
{
    // Populate private properties
    base_link_ = this->declare_parameter<std::string>("root", "base_link");
    tip_ = this->declare_parameter<std::string>("tip", "ee_link");

    distance_threshold_ = this->declare_parameter<double>("distance_threshold", 0.3);
    dangerfield_ = this->declare_parameter<double>("dangerfield", 10.0);

    double lin_limit = this->declare_parameter<double>("linearization_limit", 0.1);

    publish_mp_ = this->declare_parameter<bool>("publish_mp", true);
    publish_cmp_ = this->declare_parameter<bool>("publish_cmp", true);
    publish_vp_ = this->declare_parameter<bool>("publish_vp", true);
    publish_cvp_ = this->declare_parameter<bool>("publish_cvp", true);

    show_mp_ = this->declare_parameter<bool>("show_mp", true);
    show_cmp_ = this->declare_parameter<bool>("show_cmp", true);
    show_vp_ = this->declare_parameter<bool>("show_vp", false);
    show_cvp_ = this->declare_parameter<bool>("show_cvp", false);

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
    auto params_client = std::make_shared<rclcpp::SyncParametersClient>(this, "robot_state_publisher");
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

    tree_.getChain(base_link_, tip_, chain_);
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
    for (int i = 0; i < nseg; ++i)
    {
        KDL::Segment seg = chain_.getSegment(i);
        KDL::Joint kdl_joint = seg.getJoint();
        urdf::JointConstSharedPtr urdf_joint = model_->getJoint(kdl_joint.getName());

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

    // Create a reentrant callback group for services to allow parallel execution
    auto service_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // Instantiate ROS services and subscribers/publishers
    mesh_coll_server_ = this->create_service<arm_control::srv::AddRemoveCollisionMesh>(
        "add_remove_collision_mesh",  
        std::bind(&ConstrainedManipulabilityMod::addRemoveMeshCallback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default,
        service_callback_group);
    solid_coll_server_ = this->create_service<arm_control::srv::AddRemoveCollisionSolid>(
        "add_remove_collision_solid",  
        std::bind(&ConstrainedManipulabilityMod::addRemoveSolidCallback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default,
        service_callback_group);
    jacobian_server_ = this->create_service<arm_control::srv::GetJacobianMatrix>(
        "get_jacobian_matrix",  
        std::bind(&ConstrainedManipulabilityMod::getJacobianCallback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default,
        service_callback_group);
    update_pos_server_ = this->create_service<arm_control::srv::UpdateCollisionPose>(
        "update_collision_pose",  
        std::bind(&ConstrainedManipulabilityMod::updateCollisionObjectPoseCallback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default,
        service_callback_group);
    check_collision_pose_server_ = this->create_service<arm_control::srv::CheckCollisionPose>(
        "check_collision_pose",  
        std::bind(&ConstrainedManipulabilityMod::checkCollisionPoseCallback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default,
        service_callback_group);

    // Use reentrant callback group for subscriptions to allow parallel execution with services
    rclcpp::SubscriptionOptions joint_sub_options;
    joint_sub_options.callback_group = service_callback_group;
    rclcpp::SubscriptionOptions octo_sub_options;
    octo_sub_options.callback_group = service_callback_group;
    rclcpp::SubscriptionOptions lin_lim_sub_options;
    lin_lim_sub_options.callback_group = service_callback_group;
    
    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", rclcpp::QoS(1), 
        std::bind(&ConstrainedManipulabilityMod::jointStateCallback, this, std::placeholders::_1),
        joint_sub_options);
    octomap_filter_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
        "/octomap_filtered", rclcpp::QoS(1), 
        std::bind(&ConstrainedManipulabilityMod::octomapCallback, this, std::placeholders::_1),
        octo_sub_options);
    lin_limit_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/lin_limit", rclcpp::QoS(1), 
        std::bind(&ConstrainedManipulabilityMod::linLimitCallback, this, std::placeholders::_1),
        lin_lim_sub_options);

    coll_check_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(250),
            std::bind(&ConstrainedManipulabilityMod::checkCollisionCallback, this));

    mkr_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker", 1);
    obj_dist_pub_ = this->create_publisher<arm_control::msg::ObjectDistances>("collision/obj_distances", 1);
    filt_mesh_pub_ = this->create_publisher<octomap_filter_interfaces::msg::FilterMesh>("filter_mesh", 1);
    filt_prim_pub_ = this->create_publisher<octomap_filter_interfaces::msg::FilterPrimitive>("filter_primitive", 1);
    occupancy_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("occupancy_grid", 10);
    occupied_voxels_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("occupied_voxel_centers", rclcpp::QoS(1).transient_local());
    evaluated_voxels_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("evaluated_voxel_centers", rclcpp::QoS(1).transient_local());
    evaluated_joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::QoS(10));

    RCLCPP_INFO(this->get_logger(), "Initialized constrained_manipulability");
}

/// ROS interface methods

void ConstrainedManipulabilityMod::addRemoveMeshCallback(const std::shared_ptr<arm_control::srv::AddRemoveCollisionMesh::Request> req,
                                                      std::shared_ptr<arm_control::srv::AddRemoveCollisionMesh::Response> res)
{
    res->result = false;

    boost::mutex::scoped_lock lock(collision_world_mutex_);
    if (req->remove)
    {
        bool removed = removeCollisionObject(req->object_id);
        if (removed)
        {
            RCLCPP_INFO(this->get_logger(), "Removed mesh object ID %d from real world", req->object_id);
            res->result = true;
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Failed to remove mesh object ID %d from real world", req->object_id);
        }
    }
    else
    {
        Eigen::Affine3d mesh_T;
        tf2::fromMsg(req->pose, mesh_T);

        robot_collision_checking::FCLObjectPtr obj = std::make_shared<robot_collision_checking::FCLObject>(
            req->mesh, robot_collision_checking::MESH, mesh_T);
        bool added = addCollisionObject(obj, req->object_id);
        if (added)
        {
            RCLCPP_INFO(this->get_logger(), "Added mesh object ID %d to real world", req->object_id);
            res->result = true;
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Failed to add mesh object ID %d to real world (may already exist)", 
                        req->object_id);
        }
    }
}

void ConstrainedManipulabilityMod::addRemoveSolidCallback(const std::shared_ptr<arm_control::srv::AddRemoveCollisionSolid::Request> req,
                                                       std::shared_ptr<arm_control::srv::AddRemoveCollisionSolid::Response> res)
{
    RCLCPP_INFO(this->get_logger(), "=== Real world addRemoveSolidCallback CALLED === object_id=%d, type=%d, remove=%s",
                req->object_id, req->solid.type, req->remove ? "true" : "false");
    
    res->result = false;

    boost::mutex::scoped_lock lock(collision_world_mutex_);
    if (req->remove)
    {
        bool removed = removeCollisionObject(req->object_id);
        if (removed)
        {
            RCLCPP_INFO(this->get_logger(), "Removed collision object ID %d from real world", req->object_id);
            res->result = true;
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Failed to remove collision object ID %d from real world", req->object_id);
        }
    }
    else
    {
        Eigen::Affine3d solid_T;
        tf2::fromMsg(req->pose, solid_T);

        robot_collision_checking::FCLObjectPtr obj = std::make_shared<robot_collision_checking::FCLObject>(req->solid, solid_T);
        bool added = addCollisionObject(obj, req->object_id);
        if (added)
        {
            RCLCPP_INFO(this->get_logger(), "Added collision object ID %d (type %d) to real world", 
                        req->object_id, req->solid.type);
            res->result = true;
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Failed to add collision object ID %d to real world (may already exist)", 
                        req->object_id);
        }
    }
}

void ConstrainedManipulabilityMod::updateCollisionObjectPoseCallback(const std::shared_ptr<arm_control::srv::UpdateCollisionPose::Request> req,
                                                       std::shared_ptr<arm_control::srv::UpdateCollisionPose::Response> res)
{
    Eigen::Affine3d new_pose;
    tf2::fromMsg(req->pose, new_pose);
    res->result = collision_world_->updateCollisionObjectPose(req->object_id, new_pose);
}

void ConstrainedManipulabilityMod::checkCollisionPoseCallback(const std::shared_ptr<arm_control::srv::CheckCollisionPose::Request> req,
                                                        std::shared_ptr<arm_control::srv::CheckCollisionPose::Response> res)
{
    try 
    {
        bool self_col, env_col;
        res->in_collision = evaluateCollisionState(req->joint_state, self_col, env_col);

        sensor_msgs::msg::JointState prefixed_state;
        prefixed_state.header.stamp = this->now();
        prefixed_state.position = req->joint_state.position;
        prefixed_state.velocity = req->joint_state.velocity;
        prefixed_state.effort = req->joint_state.effort;

        for (const auto& name : req->joint_state.name) {
            prefixed_state.name.push_back("collision_" + name);
        }

        evaluated_joint_pub_->publish(prefixed_state);
    } 
    catch (const std::exception& e) 
    {
        RCLCPP_ERROR(this->get_logger(), "Error in checkCollisionPoseCallback: %s", e.what());
        res->in_collision = false;  // opción conservadora
    }
}


void ConstrainedManipulabilityMod::getJacobianCallback(const std::shared_ptr<arm_control::srv::GetJacobianMatrix::Request> req,
                                                    std::shared_ptr<arm_control::srv::GetJacobianMatrix::Response> res)
{
    Eigen::Matrix<double, 6, Eigen::Dynamic> base_J_ee;
    getJacobian(req->joint_state, base_J_ee);
    res->jacobian = eigenToMatrix(base_J_ee);
}

void ConstrainedManipulabilityMod::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    joint_state_mutex_.lock();
    joint_state_ = *msg;
    KDL::JntArray kdl_joint_positions(ndof_);
    jointStatetoKDLJointArray(chain_, joint_state_, kdl_joint_positions);
    joint_state_mutex_.unlock();

    GeometryInformation geometry_information;
    getCollisionModel(kdl_joint_positions, geometry_information);
    addBranchCollisionGeometry(kdl_joint_positions, geometry_information);

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

void ConstrainedManipulabilityMod::octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg)
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

void ConstrainedManipulabilityMod::generateOccupancyGrid()
{
    boost::mutex::scoped_lock lock(collision_world_mutex_);

    const double resolution = 0.05;  // tamaño del voxel
    const double half_res = resolution / 2.0;

    std::set<std::tuple<int, int, int>> occupied_cells;
    std::vector<Eigen::Vector3d> all_voxel_centers;

    for (const auto& obj : collision_world_->getCollisionObjects())
    {
        if (!obj)
            continue;

        const fcl::AABBd& aabb = obj->collision_object->getAABB();

        double x_start = std::floor(aabb.min_.x() / resolution) * resolution - half_res;
        double y_start = std::floor(aabb.min_.y() / resolution) * resolution - half_res;
        double z_start = std::floor(aabb.min_.z() / resolution) * resolution - half_res;

        double x_end = std::ceil(aabb.max_.x() / resolution) * resolution + half_res;
        double y_end = std::ceil(aabb.max_.y() / resolution) * resolution + half_res;
        double z_end = std::ceil(aabb.max_.z() / resolution) * resolution + half_res;

        for (double x = x_start; x <= x_end; x += resolution)
        {
            for (double y = y_start; y <= y_end; y += resolution)
            {
                for (double z = z_start; z <= z_end; z += resolution)
                {
                    Eigen::Vector3d voxel_center(x + half_res, y + half_res, z + half_res);
                    all_voxel_centers.push_back(voxel_center);
                    fcl::Transform3d tf = fcl::Transform3d::Identity();
                    tf.translation() = voxel_center;

                    auto voxel_shape = std::make_shared<fcl::Boxd>(resolution, resolution, resolution);
                    fcl::CollisionObjectd voxel_box(voxel_shape, tf);

                    fcl::DistanceRequestd req(true);  // enable signed distance
                    fcl::DistanceResultd res;
                    double distance = fcl::distance(&voxel_box, obj->collision_object.get(), req, res);

                    // Tolerancia mayor para considerar interiores
                    if (distance < resolution * 0.75)  // por ejemplo 75% del tamaño del voxel
                    {
                        int xi = static_cast<int>(std::floor(x / resolution));
                        int yi = static_cast<int>(std::floor(y / resolution));
                        int zi = static_cast<int>(std::floor(z / resolution));
                        occupied_cells.emplace(xi, yi, zi);
                    }
                    if (distance > 0.5 * resolution && distance < 1.5 * resolution)
                    {
                        // RCLCPP_INFO(this->get_logger(), "Near miss at voxel (%.2f, %.2f, %.2f), distance = %.4f",
                        //             voxel_center.x(), voxel_center.y(), voxel_center.z(), distance);
                    }
                }
            }
        }
    }

    // RCLCPP_INFO(this->get_logger(), "Total occupied voxels: %zu", occupied_cells.size());
    publishVoxelMarkers(occupied_cells, resolution);

    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header.stamp = this->now();
    cloud_msg.header.frame_id = "base_link";  // o el frame correcto

    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(occupied_cells.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

    for (const auto& cell : occupied_cells) {
        geometry_msgs::msg::Point32 pt;
        pt.x = (static_cast<float>(std::get<0>(cell)) + 0.5f) * resolution;
        pt.y = (static_cast<float>(std::get<1>(cell)) + 0.5f) * resolution;
        pt.z = (static_cast<float>(std::get<2>(cell)) + 0.5f) * resolution;
        *iter_x = pt.x;
        *iter_y = pt.y;
        *iter_z = pt.z;
        ++iter_x;
        ++iter_y;
        ++iter_z;
    }

    occupied_voxels_pub_->publish(cloud_msg);

    sensor_msgs::msg::PointCloud2 eval_cloud;
    eval_cloud.header.stamp = this->now();
    eval_cloud.header.frame_id = "base_link";

    sensor_msgs::PointCloud2Modifier eval_modifier(eval_cloud);
    eval_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
    eval_modifier.resize(all_voxel_centers.size());

    sensor_msgs::PointCloud2Iterator<float> ex(eval_cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> ey(eval_cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> ez(eval_cloud, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> er(eval_cloud, "rgb");

    for (const auto& pt : all_voxel_centers) {
        *ex = pt.x();
        *ey = pt.y();
        *ez = pt.z();
        uint8_t r = 255, g = 0, b = 0;
        uint32_t rgb = (r << 16) | (g << 8) | b;
        *er = *reinterpret_cast<float*>(&rgb);
        ++ex; ++ey; ++ez; ++er;
    }

    evaluated_voxels_pub_->publish(eval_cloud);

}

void ConstrainedManipulabilityMod::publishVoxelMarkers(const std::set<std::tuple<int, int, int>>& occupied_cells, double resolution)
{
    visualization_msgs::msg::MarkerArray marker_array;
    int id = 0;

    for (const auto& [xi, yi, zi] : occupied_cells)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";  // usa tu frame adecuado
        marker.header.stamp = this->now();
        marker.ns = "occupied_voxels";
        marker.id = id++;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = (xi + 0.5) * resolution;
        marker.pose.position.y = (yi + 0.5) * resolution;
        marker.pose.position.z = (zi + 0.5) * resolution;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = resolution;
        marker.scale.y = resolution;
        marker.scale.z = resolution;
        marker.color.r = 0.0f;
        marker.color.g = 0.8f;
        marker.color.b = 0.1f;
        marker.color.a = 0.5f;
        marker.lifetime = rclcpp::Duration::from_seconds(1.0);  // permanente

        marker_array.markers.push_back(marker);
    }

    occupancy_pub_->publish(marker_array);
}


void ConstrainedManipulabilityMod::linLimitCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
    setLinearizationLimit(msg->data);    
}

void ConstrainedManipulabilityMod::checkCollisionCallback()
{
    // joint_state_mutex_.lock();
    // sensor_msgs::msg::JointState curr_joint_state = joint_state_;
    // joint_state_mutex_.unlock();

    // // checkCollision(curr_joint_state);    // opcion original

    // bool collision_detected = checkCollision(curr_joint_state);  // opcion propia
    // if (collision_detected)
    // {
    //     RCLCPP_WARN(this->get_logger(), "¡Colisión detectada!");
    // }
    // else
    // {
    //     RCLCPP_WARN(this->get_logger(), "Sin colisión");
    // }  

    // static bool last_collision_state = false;
    // rclcpp::Logger logger = this->get_logger();

    // joint_state_mutex_.lock();
    // sensor_msgs::msg::JointState curr_joint_state = joint_state_;
    // joint_state_mutex_.unlock();
    
    // bool collision_detected = checkCollision(curr_joint_state);

    // if (collision_detected != last_collision_state)
    // {
    //     last_collision_state = collision_detected;
    //     if (collision_detected)
    //     {
    //         RCLCPP_WARN(logger, "¡Colisión detectada!");
    //     }
    //     else
    //     {
    //         RCLCPP_WARN(logger, "Sin colisión");
    //     }
    // }

    joint_state_mutex_.lock();
    sensor_msgs::msg::JointState curr_joint_state = joint_state_;
    joint_state_mutex_.unlock();


    bool collision_detected = false;

    try
    {
        collision_detected = checkCollision(curr_joint_state);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Exception in checkCollision: %s", e.what());
        return;
    }
    catch (...)
    {
        RCLCPP_ERROR(this->get_logger(), "Unknown exception in checkCollision");
        return;
    }

    if (collision_detected != last_collision_state_)
    {
        last_collision_state_ = collision_detected;
        if (collision_detected)
        {
            RCLCPP_WARN(this->get_logger(), "Collision detected!");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "All collisions cleared");
        }
    }
}

/// Private member methods (excluding ROS callbacks)

bool ConstrainedManipulabilityMod::addCollisionObject(const robot_collision_checking::FCLObjectPtr& obj, int object_id)
{
    return collision_world_->addCollisionObject(obj, object_id);
}

bool ConstrainedManipulabilityMod::removeCollisionObject(int object_id)
{
    return collision_world_->removeCollisionObject(object_id);
}

bool ConstrainedManipulabilityMod::checkCollision(const sensor_msgs::msg::JointState& joint_state)
{
    KDL::JntArray kdl_joint_positions(ndof_);
    jointStatetoKDLJointArray(chain_, joint_state, kdl_joint_positions);

    GeometryInformation geometry_information;
    getCollisionModel(kdl_joint_positions, geometry_information);
    std::vector<std::string> branch_link_names = addBranchCollisionGeometry(kdl_joint_positions, geometry_information);

    std::vector<shapes::ShapeMsg> current_shapes;
    std::vector<geometry_msgs::msg::Pose> shapes_poses;
    convertCollisionModel(geometry_information, current_shapes, shapes_poses);

    // Build collision geometry for robot if not yet created
    // Assume collision geometry for robot's meshes do not change
    if (robot_collision_geometry_.size() == 0)
    {
        createRobotCollisionModel(geometry_information, branch_link_names);
        buildCollisionExclusions();
    }
    
    boost::mutex::scoped_lock lock(collision_world_mutex_);

    // Self-collision check
    bool self_collision_detected = checkSelfCollision(geometry_information);
    if (self_collision_detected != last_self_collision_state_)
    {
        last_self_collision_state_ = self_collision_detected;
        if (self_collision_detected)
        {
            RCLCPP_WARN(this->get_logger(), "Self-collision detected!");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Self-collision cleared.");
        }
    }

    // External environement collision check
    bool env_collision_detected = false;
    int num_shapes = geometry_information.shapes.size();
    for (int i = 0; i < num_shapes; i++)
    {
        Eigen::Affine3d obj_pose;
        tf2::fromMsg(shapes_poses[i], obj_pose);
        std::vector<int> collision_object_ids;
        if (current_shapes[i].which() == 0)
        {
            robot_collision_checking::FCLObjectPtr obj = std::make_shared<robot_collision_checking::FCLObject>(
                boost::get<shape_msgs::msg::SolidPrimitive>(current_shapes[i]), obj_pose);

            fcl::Transform3d world_to_fcl;
            robot_collision_checking::fcl_interface::transform2fcl(obj->object_transform, world_to_fcl);
            robot_collision_checking::FCLCollisionObjectPtr co = std::make_shared<fcl::CollisionObjectd>(robot_collision_geometry_[i], world_to_fcl);
            
            if (collision_world_->checkCollisionObject(co, collision_object_ids))
            {
                env_collision_detected = true;
            }
        }
        else if (current_shapes[i].which() == 1)
        {
            robot_collision_checking::FCLObjectPtr obj = std::make_shared<robot_collision_checking::FCLObject>(
                boost::get<shape_msgs::msg::Mesh>(current_shapes[i]), robot_collision_checking::MESH, obj_pose);

            fcl::Transform3d world_to_fcl;
            robot_collision_checking::fcl_interface::transform2fcl(obj->object_transform, world_to_fcl);
            robot_collision_checking::FCLCollisionObjectPtr co = std::make_shared<fcl::CollisionObjectd>(robot_collision_geometry_[i], world_to_fcl);

            if (collision_world_->checkCollisionObject(co, collision_object_ids))
            {
                env_collision_detected = true;
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Collision geometry not supported");
        }
    }

    if (env_collision_detected != last_env_collision_state_)
    {
        last_env_collision_state_ = env_collision_detected;
        if (env_collision_detected)
        {
            RCLCPP_WARN(this->get_logger(), "Environement collision detected!");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Environement collision cleared.");
        }
    }
    
    return env_collision_detected || self_collision_detected;
}

bool ConstrainedManipulabilityMod::evaluateCollisionState(const sensor_msgs::msg::JointState& joint_state, bool& self_collision, bool& env_collision)
{
    KDL::JntArray kdl_joint_positions(ndof_);
    jointStatetoKDLJointArray(chain_, joint_state, kdl_joint_positions);

    GeometryInformation geometry_information;
    getCollisionModel(kdl_joint_positions, geometry_information);
    std::vector<std::string> branch_link_names = addBranchCollisionGeometry(kdl_joint_positions, geometry_information);

    std::vector<shapes::ShapeMsg> current_shapes;
    std::vector<geometry_msgs::msg::Pose> shapes_poses;
    convertCollisionModel(geometry_information, current_shapes, shapes_poses);

    if (robot_collision_geometry_.size() == 0)
    {
        createRobotCollisionModel(geometry_information, branch_link_names);
        buildCollisionExclusions();
    }

    boost::mutex::scoped_lock lock(collision_world_mutex_);

    self_collision = checkSelfCollision(geometry_information);

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


bool ConstrainedManipulabilityMod::checkSelfCollision(const GeometryInformation& geometry_information)
{
    bool any_collision_active = false;
    int n = geometry_information.shapes.size();
    int n_names = static_cast<int>(collision_link_names_.size());

    for (int i = 0; i < n; ++i)
    {
        Eigen::Isometry3d pose_i;
        pose_i.linear() = geometry_information.geometry_transforms[i].linear();
        pose_i.translation() = geometry_information.geometry_transforms[i].translation();
        fcl::CollisionObjectd obj_i(robot_collision_geometry_[i], pose_i);

        for (int j = i + 1; j < n; ++j)
        {
            // Opcional: ignora pares adyacentes aquí si quieres
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
                    // Get link names for better debugging - safe access to collision_link_names_
                    std::string link_i_name = (i < n_names) ? collision_link_names_[i] : "link_" + std::to_string(i);
                    std::string link_j_name = (j < n_names) ? collision_link_names_[j] : "link_" + std::to_string(j);
                    RCLCPP_WARN(this->get_logger(), "Self-collision detected between %s and %s", 
                                link_i_name.c_str(), link_j_name.c_str());
                    return true;
                }
                else
                {
                    std::string link_i_name = (i < n_names) ? collision_link_names_[i] : "link_" + std::to_string(i);
                    std::string link_j_name = (j < n_names) ? collision_link_names_[j] : "link_" + std::to_string(j);
                    RCLCPP_INFO(this->get_logger(), "Self-collision cleared between %s and %s", 
                                link_i_name.c_str(), link_j_name.c_str());
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

bool ConstrainedManipulabilityMod::isAdjacent(int i, int j) const
{
    // Check if pair is in the exclusion set
    auto key1 = std::make_pair(i, j);
    auto key2 = std::make_pair(j, i);  // Check both orders
    return (collision_exclusions_.find(key1) != collision_exclusions_.end()) ||
           (collision_exclusions_.find(key2) != collision_exclusions_.end());
}

void ConstrainedManipulabilityMod::buildCollisionExclusions()
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

void ConstrainedManipulabilityMod::displayCollisionModel(const GeometryInformation& geometry_information, const Eigen::Vector4d& color)
{
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
    
    std::vector<robot_collision_checking::FCLInterfaceCollisionObjectPtr> world_objects = collision_world_->getCollisionObjects();
    int num_objects = collision_world_->getNumObjects();

    // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
    //                     "Real world has %d collision objects to visualize", num_objects);

    for (int i = 0; i < num_objects; /*i++*/)
    {
        auto world_obj = world_objects[i];
        // Make a marker
        visualization_msgs::msg::Marker mkr;
        mkr.ns = "collision_objects";
        mkr.header.frame_id = base_link_;
        mkr.action = visualization_msgs::msg::Marker::ADD;
        mkr.lifetime = rclcpp::Duration(1, 0);
        std::string obj_type = world_obj->object->getTypeString();

        // Get object pose relative to world_frame
        Eigen::Affine3d object_eig_pose = world_obj->object->object_transform;
        geometry_msgs::msg::Pose object_geo_pose;
        robot_collision_checking::fcl_interface::convertEigenTransformGeometryPose(object_eig_pose, object_geo_pose);
        mkr.pose = object_geo_pose;

        if (obj_type == "MESH")
        {
            geometric_shapes::constructMarkerFromShape(*(world_obj->object->ptr.mesh), mkr);

            // Blue mesh
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
            mkr.scale.z = 0.001; // very thin

            // Red cuboid
            mkr.type = visualization_msgs::msg::Marker::CUBE;
            mkr.id = world_obj->collision_id;
            mkr.color.r = 1.0;
            mkr.color.a = 0.3;
            marker_array_msg->markers.push_back(mkr);
            i++;
        }
        else if (obj_type == "OCTOMAP")
        {
            // RCLCPP_WARN(this->get_logger(), "Unable to display octomap");
            i++;
        }
        else if (obj_type == "VOXEL_GRID")
        {
            mkr.id = world_obj->collision_id;
            // Treat voxel grid as a cube list
            mkr.type = visualization_msgs::msg::Marker::CUBE_LIST;
            auto grid = *(world_obj->object->ptr.voxel_grid);
            mkr.scale.x = grid.resolutions.x;
            mkr.scale.y = grid.resolutions.y;
            mkr.scale.z = grid.resolutions.z;
            // Voxel cells already account for position in world
            mkr.pose.position.x = mkr.pose.position.y = mkr.pose.position.z = 0.0;

            // Iterate over remaining cells until no more objects in world or a new collision object
            do
            {
                // The collision object is really a voxel cell
                auto voxel_cell = *(world_objects[i]->collision_object);
                fcl::Vector3d cell_center = voxel_cell.getTranslation();
                // Invert rotation to obtain cell position in original world frame
                Eigen::Matrix3d rotation_matrix = voxel_cell.getRotation().matrix();
                rotation_matrix.transposeInPlace();
                cell_center = rotation_matrix * cell_center;
                geometry_msgs::msg::Point point;
                point.x = cell_center[0];
                point.y = cell_center[1];
                point.z = cell_center[2];
                mkr.points.push_back(point);
                i++;
            } while ((i < num_objects) && (world_objects[i-1]->collision_id == world_objects[i]->collision_id));

            // Purple voxel grid
            mkr.color.r = 1.0;
            mkr.color.b = 1.0;
            mkr.color.a = 1.0;
            marker_array_msg->markers.push_back(mkr);
        }
        else if (obj_type == "SPHERE" || obj_type == "BOX" || obj_type == "CYLINDER" || obj_type == "CONE")
        {
            geometric_shapes::constructMarkerFromShape(*(world_obj->object->ptr.solid), mkr);

            // Green primitives
            mkr.id = world_obj->collision_id;
            mkr.color.g = 1.0;
            mkr.color.a = 1.0;
            
            RCLCPP_DEBUG(this->get_logger(), "Visualizing %s object ID %d with scale [%.3f, %.3f, %.3f]",
                        obj_type.c_str(), mkr.id, mkr.scale.x, mkr.scale.y, mkr.scale.z);
            
            marker_array_msg->markers.push_back(mkr);
            i++;
        }
    }

    mkr_pub_->publish(*marker_array_msg);

    generateOccupancyGrid();
}

void ConstrainedManipulabilityMod::convertCollisionModel(const GeometryInformation& geometry_information,
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

void ConstrainedManipulabilityMod::createRobotCollisionModel(const GeometryInformation& geometry_information, 
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

std::unique_ptr<shapes::Shape> ConstrainedManipulabilityMod::constructShape(const urdf::Geometry* geom) const
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

void ConstrainedManipulabilityMod::getKDLKinematicInformation(const KDL::JntArray& kdl_joint_positions, Eigen::Affine3d& T,
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

void ConstrainedManipulabilityMod::getCollisionModel(const KDL::JntArray& kdl_joint_positions, GeometryInformation& geometry_information) const
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

std::vector<std::string> ConstrainedManipulabilityMod::addBranchCollisionGeometry(const KDL::JntArray& joint_positions, GeometryInformation& geometry_information) const
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

        // For branch links, we use a zero Jacobian since they don't contribute to polytope calculations
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

void ConstrainedManipulabilityMod::getJacobian(const sensor_msgs::msg::JointState& joint_state, Eigen::Matrix<double, 6, Eigen::Dynamic>& Jac) const
{
    KDL::JntArray kdl_joint_positions(ndof_);
    Eigen::Affine3d base_T_ee;
    jointStatetoKDLJointArray(chain_, joint_state, kdl_joint_positions);
    getKDLKinematicInformation(kdl_joint_positions, base_T_ee, Jac);
}

void ConstrainedManipulabilityMod::getTransform(const sensor_msgs::msg::JointState& joint_state, Eigen::Affine3d& T) const
{
    KDL::JntArray kdl_joint_positions(ndof_);
    Eigen::Matrix<double, 6, Eigen::Dynamic> Jac;
    jointStatetoKDLJointArray(chain_, joint_state, kdl_joint_positions);
    getKDLKinematicInformation(kdl_joint_positions, T, Jac);
}

void ConstrainedManipulabilityMod::getCartPos(const sensor_msgs::msg::JointState& joint_state, geometry_msgs::msg::Pose& geo_pose) const
{
    KDL::JntArray kdl_joint_positions(ndof_);
    jointStatetoKDLJointArray(chain_, joint_state, kdl_joint_positions);

    KDL::Frame cartpos;
    kdl_fk_solver_->JntToCart(kdl_joint_positions, cartpos);

    Eigen::Affine3d T;
    tf2::transformKDLToEigen(cartpos, T);
    geo_pose = tf2::toMsg(T);
}
} // namespace constrained_manipulability