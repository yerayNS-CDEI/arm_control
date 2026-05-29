#include "collision/constrained_manipulability_utils.hpp"

namespace constrained_manipulability
{
namespace {
// Skew symmetric matrix performing the cross product
inline bool skew(const Eigen::Vector3d& L, Eigen::Matrix<double, 3, 3>& skewL)
{
    skewL(0, 1) = -L(2);
    skewL(0, 2) = L(1);
    skewL(1, 2) = -L(0);
    skewL(1, 0) = -skewL(0, 1);
    skewL(2, 0) = -skewL(0, 2);
    skewL(2, 1) = -skewL(1, 2);
    return true;
}
} // namespace

// Screw transform to move a twist from point a to point b, given vector L, a->b w.r.t. base frame
bool screwTransform(const Eigen::Matrix<double, 6, Eigen::Dynamic>& J0N_in,
                    const Eigen::Vector3d& L, 
                    Eigen::Matrix<double, 6, Eigen::Dynamic>& J0E_out)
{
    J0E_out.setZero();
    Eigen::Matrix<double, 3, 3> Lhat;
    Lhat.setZero();
    skew(L, Lhat);
    Eigen::Matrix<double, 6, 6> screwL;
    screwL.setZero();
    screwL.topLeftCorner(3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
    screwL.bottomRightCorner(3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
    screwL.bottomLeftCorner(3, 3) = Eigen::Matrix<double, 3, 3>::Zero();
    screwL.topRightCorner(3, 3) = -Lhat;
    J0E_out = screwL * J0N_in;
    return true;
}

void getCollisionShapes(const std::vector<int>& object_primitive, 
                        const std::vector<std::vector<double>>& obj_dimensions,
                        const std::vector<std::vector<double>>& obj_poses,
                        std::vector<shape_msgs::msg::SolidPrimitive>& shapes,
                        TransformVector& shape_poses)
{
    unsigned int obj_primitive_size = object_primitive.size();
    assert(obj_dimensions.size() == obj_primitive_size);
    assert(obj_poses.size() == obj_primitive_size);

    shapes.resize(obj_primitive_size);
    shape_poses.resize(obj_primitive_size);

    for (unsigned int i = 0; i < obj_primitive_size; ++i)
    {
        shapes[i].type = object_primitive[i];
        shapes[i].dimensions.resize(obj_dimensions[i].size());
        for (unsigned int j = 0; j < obj_dimensions[i].size(); ++j)
        {
            shapes[i].dimensions[j] = obj_dimensions[i][j];
        }
        Eigen::Vector3d p(obj_poses[i][0], obj_poses[i][1], obj_poses[i][2]);
        Eigen::Quaterniond q(obj_poses[i][3], obj_poses[i][4], obj_poses[i][5], obj_poses[i][6]);
        q.normalize();
        shape_poses[i].translation() = p;
        shape_poses[i].linear() = q.toRotationMatrix();
    }
}

// Convert a joint state message to a KDL joint array based on segment names
void jointStatetoKDLJointArray(const KDL::Chain& chain,
                               const sensor_msgs::msg::JointState& joint_state, 
                               KDL::JntArray& kdl_joint_positions)
{
    unsigned int jnt(0);
    unsigned int num_segments(chain.getNrOfSegments());
    for (unsigned int i = 0; i < num_segments; ++i)
    {
        KDL::Segment seg = chain.getSegment(i);
        KDL::Joint kdl_joint = seg.getJoint();
        for (unsigned int j = 0; j < joint_state.name.size(); ++j)
        {
            if (kdl_joint.getName() == joint_state.name[j])
            {
                kdl_joint_positions(jnt) = joint_state.position[j];
                jnt++;
            }
        }
    }
}

// Convert joint state to KDL joint array for the full tree (all joints)
// Helper function to recursively walk tree and collect joints in kinematic order
void walkTreeForJoints(const KDL::SegmentMap::const_iterator& segment_it,
                       const KDL::SegmentMap& segment_map,
                       std::vector<std::string>& joint_names,
                       const rclcpp::Logger& logger,
                       bool log_output)
{
    const KDL::Segment& segment = segment_it->second.segment;
    const KDL::Joint& joint = segment.getJoint();
    
    // Add this joint if it's not a fixed joint
    if (joint.getType() != KDL::Joint::None)
    {
        joint_names.push_back(joint.getName());
        if (log_output)
        {
            RCLCPP_INFO(logger, "Tree joint[%zu] = '%s' (kinematic order)", 
                       joint_names.size() - 1, joint.getName().c_str());
        }
    }
    
    // Recursively process all children
    for (const auto& child_it : segment_it->second.children)
    {
        walkTreeForJoints(child_it, segment_map, joint_names, logger, log_output);
    }
}

void jointStatetoKDLTreeJointArray(const KDL::Tree& tree,
                                    const sensor_msgs::msg::JointState& joint_state,
                                    KDL::JntArray& kdl_joint_positions)
{
    static rclcpp::Logger logger = rclcpp::get_logger("jointStatetoKDLTreeJointArray");
    static bool first_call = true;
    
    // Walk the tree from root to collect joints in kinematic order
    KDL::SegmentMap segment_map = tree.getSegments();
    std::vector<std::string> tree_joint_names;
    
    // Start from root segment
    auto root_it = segment_map.find(tree.getRootSegment()->first);
    if (root_it != segment_map.end())
    {
        walkTreeForJoints(root_it, segment_map, tree_joint_names, logger, first_call);
    }
    
    // Build joint name to index mapping
    std::map<std::string, unsigned int> joint_name_to_index;
    for (unsigned int i = 0; i < tree_joint_names.size(); ++i)
    {
        joint_name_to_index[tree_joint_names[i]] = i;
    }
    
    // Fill joint positions from joint_state
    for (unsigned int i = 0; i < joint_state.name.size(); ++i)
    {
        auto it = joint_name_to_index.find(joint_state.name[i]);
        if (it != joint_name_to_index.end())
        {
            kdl_joint_positions(it->second) = joint_state.position[i];
            if (first_call)
            {
                RCLCPP_INFO(logger, "  Mapped joint_state['%s'] = %.3f -> kdl_tree[%u]", 
                           joint_state.name[i].c_str(), joint_state.position[i], it->second);
            }
        }
    }
    
    first_call = false;
}

// Project translational Jacobian matrix along a vector
bool projectTranslationalJacobian(const Eigen::Vector3d& nT,
                                  const Eigen::Matrix<double, 6, Eigen::Dynamic>& J0N_in,
                                  Eigen::Matrix<double, 1, Eigen::Dynamic>& J0N_out)
{
    int n = J0N_in.cols();
    assert(J0N_in.rows() > 3);
    J0N_out.setZero();
    J0N_out = nT.transpose() * J0N_in.topLeftCorner(3, n);
    return true;
}
} // namespace constrained_manipulability