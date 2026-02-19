from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace, Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    # --- Package and child launch paths ---
    ur_pkg = FindPackageShare('arm_control')
    ur_sim_control_launch = PathJoinSubstitution([ur_pkg, 'launch', 'ur_sim_control.launch.py'])
    ur_control_launch = PathJoinSubstitution([ur_pkg, 'launch', 'ur_control.launch.py'])
    publisher_launch = PathJoinSubstitution([ur_pkg, 'launch', 'test_scaled_joint_trajectory_planned.launch.py'])

    # --- Parent-level args (edit at CLI if needed) ---
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.1.102',
        description='IP for ARM UR robot',
    )

    use_fake_hardware_arg = DeclareLaunchArgument(
        'use_fake_hardware',
        default_value='false',
        description='Whether to use fake hardware in child launches',
    )

    tf_prefix_arg = DeclareLaunchArgument(
        'tf_prefix',
        default_value='arm_',
        # default_value='',
        description='The prefix to use for the TF tree',
    )
    
    prefix_arg = DeclareLaunchArgument(
        'prefix',
        default_value='arm_',
        # default_value='',
        description='The prefix to use for the TF tree',
    )

    # Optionally toggle RViz from the parent (usually off to avoid duplicate RViz instances)
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Open RViz from child launch files',
    )

    # UR types (adjust as desired)
    ur_type_arg = DeclareLaunchArgument(
        'ur_type',
        default_value='ur10e',
        description='UR type for the arm robot',
    )
    
    # 
    simulation_arg = DeclareLaunchArgument(
        'sim',
        default_value='false',
        description='Whether to run in simulation mode',
    )
    
    mode_arg = DeclareLaunchArgument(
        "mode",
        default_value="full",
        description="Launch mode full|arm",
        choices=['full', 'arm'],
    )
    
    namespace_arm_arg = DeclareLaunchArgument(
        'namespace_arm',
        default_value='',
        description='Namespace for the arm robot',
    )
    
    # Use LaunchConfiguration values
    robot_ip = LaunchConfiguration('robot_ip')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    tf_prefix = LaunchConfiguration('tf_prefix')
    prefix = LaunchConfiguration('prefix')
    launch_rviz = LaunchConfiguration('launch_rviz')
    ur_type = LaunchConfiguration('ur_type')
    simulation = LaunchConfiguration('sim')
    mode = LaunchConfiguration('mode')
    namespace_arm = LaunchConfiguration('namespace_arm')

    # --- Namespaced groups (namespace ONLY in the parent) ---
    arm_group = GroupAction([
        # Include the UR control stack for the arm
        PushRosNamespace(namespace_arm),
        # IMPORTANT: we DO NOT pass a 'namespace' arg to the child.
        # The parent namespace applies to everything inside this group.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ur_control_launch),
            launch_arguments={
                'ur_type':              ur_type,
                'robot_ip':             robot_ip,
                'use_fake_hardware':    use_fake_hardware,
                'tf_prefix':            tf_prefix,
                'prefix':               prefix,
                'mode':                 mode,
            }.items(),
            condition=UnlessCondition(simulation)
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ur_sim_control_launch),
            launch_arguments={
                'ur_type':              ur_type,
                'tf_prefix':            tf_prefix,
                'prefix':               prefix,
                'mode':                 mode,
                'launch_rviz':          launch_rviz,
            }.items(),
            condition=IfCondition(simulation)
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(publisher_launch),
            launch_arguments={
                'check_starting_point': 'false',        #! TODO: Check if in the real robot a safe starting position wants to be added (folding sequence whenever closing the robot)
            }.items(),
        ),

        Node(
            package="arm_control",
            executable="planner_node",
            name="robotic_arm_planner_node",
            output="screen",
        ),

        Node(
            package="arm_control",
            executable="end_effector_pose_node",
            name="end_effector_pose_node",
            output="screen",
        ),        
    ])

    return LaunchDescription([
        robot_ip_arg,
        use_fake_hardware_arg,
        tf_prefix_arg,
        prefix_arg,
        launch_rviz_arg,
        ur_type_arg,
        simulation_arg,
        mode_arg,
        namespace_arm_arg,
        arm_group,
    ])
