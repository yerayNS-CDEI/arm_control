from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace, Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # --- Package and child launch paths ---
    ur_pkg = FindPackageShare('ur_arm_control')
    ur_control_launch = PathJoinSubstitution([ur_pkg, 'launch', 'ur_control.launch.py'])
    publisher_launch = PathJoinSubstitution([ur_pkg, 'launch', 'test_scaled_joint_trajectory_planned.launch.py'])

    # --- Parent-level args (edit at CLI if needed) ---
    arm_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value=TextSubstitution(text='192.168.56.101'),
        description='IP for ARM UR robot',
    )

    use_fake_arg = DeclareLaunchArgument(
        'use_fake_hardware',
        default_value=TextSubstitution(text='true'),
        description='Whether to use fake hardware in child launches',
    )

    tf_prefix_arg = DeclareLaunchArgument(
        'tf_prefix',
        # default_value=TextSubstitution(text='arm_'),
        default_value=TextSubstitution(text=''),
        description='The prefix to use for the TF tree',
    )

    # Optionally toggle RViz from the parent (usually off to avoid duplicate RViz instances)
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value=TextSubstitution(text='true'),
        description='Open RViz from child launch files',
    )

    # UR types (adjust as desired)
    arm_type_arg = DeclareLaunchArgument(
        'ur_type',
        default_value=TextSubstitution(text='ur10e'),
        description='UR type for the arm robot',
    )

    # ----- Sim time param (configurable desde CLI) -----
    arm_use_sim_time_arg = DeclareLaunchArgument(
        'arm_use_sim_time',
        default_value=TextSubstitution(text='false'),
        description='Use /clock (sim time) for /arm namespace',
    )
    arm_use_sim_time = LaunchConfiguration('arm_use_sim_time')
    

    # --- Namespaced groups (namespace ONLY in the parent) ---
    arm_group = GroupAction([
        PushRosNamespace('arm'),
        SetParameter(name='use_sim_time', value=arm_use_sim_time),
        # Include the UR control stack for the arm
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ur_control_launch),
            # IMPORTANT: we DO NOT pass a 'namespace' arg to the child.
            # The parent namespace applies to everything inside this group.
            launch_arguments={
                'ur_type':            LaunchConfiguration('ur_type'),
                'robot_ip':           LaunchConfiguration('robot_ip'),
                'use_fake_hardware':  LaunchConfiguration('use_fake_hardware'),
                # 'tf_prefix':          LaunchConfiguration('tf_prefix'),
                'launch_rviz':        LaunchConfiguration('launch_rviz'),
                'controllers_file':   TextSubstitution(text='ur_controllers_namespace.yaml'),
            }.items(),
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(publisher_launch),
            launch_arguments={
                'check_starting_point': TextSubstitution(text='false'), # We do not pass 'namespace' here; it inherits '/arm' from GroupAction.
            }.items(),
        ),

        Node(
            package="robotic_arm_planner",
            executable="planner_node",
            name="robotic_arm_planner_node",
            output="screen",
            parameters=[{'use_sim_time': arm_use_sim_time}],
        ),

        Node(
            package="ur_arm_control",
            executable="end_effector_pose_node",
            name="end_effector_pose_node",
            output="screen",
            parameters=[{'use_sim_time': arm_use_sim_time}],
        ),
        
    ])

    return LaunchDescription([
        arm_ip_arg,
        use_fake_arg,
        tf_prefix_arg,
        launch_rviz_arg,
        arm_type_arg,
        arm_use_sim_time_arg,
        arm_group,
    ])
