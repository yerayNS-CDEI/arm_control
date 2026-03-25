from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    AndSubstitution,
    LaunchConfiguration,
    NotSubstitution,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_pkg = FindPackageShare('arm_control')
    ur_sim_control_launch = PathJoinSubstitution([ur_pkg, 'launch', 'ur_sim_control.launch.py'])
    ur_control_launch = PathJoinSubstitution([ur_pkg, 'launch', 'ur_control.launch.py'])
    publisher_launch = PathJoinSubstitution(
        [ur_pkg, 'launch', 'test_scaled_joint_trajectory_planned.launch.py']
    )
    moveit_launch = PathJoinSubstitution([ur_pkg, 'launch', 'arm_moveit.launch.py'])

    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.56.101',
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
        description='The prefix to use for the TF tree',
    )
    prefix_arg = DeclareLaunchArgument(
        'prefix',
        default_value='arm_',
        description='The prefix to use for the TF tree',
    )
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Open RViz from child launch files',
    )
    ur_type_arg = DeclareLaunchArgument(
        'ur_type',
        default_value='ur10e',
        description='UR type for the arm robot',
    )
    simulation_arg = DeclareLaunchArgument(
        'sim',
        default_value='false',
        description='Whether to run in simulation mode',
    )
    hybrid_sim_arg = DeclareLaunchArgument(
        'hybrid_sim',
        default_value='true',
        description=(
            'If true launch ur_control stack (hybrid mode). '
            'If false, ur_sim_control can be launched when sim=true.'
        ),
    )
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='full',
        description='Launch mode full|arm',
        choices=['full', 'arm'],
    )
    moveit_mode_arg = DeclareLaunchArgument(
        'moveit_mode',
        default_value='auto',
        description='MoveIt robot mode auto|full|arm',
        choices=['auto', 'full', 'arm'],
    )
    moveit_use_sim_time_arg = DeclareLaunchArgument(
        'moveit_use_sim_time',
        default_value='auto',
        description='MoveIt clock source auto|true|false',
        choices=['auto', 'true', 'false'],
    )
    controllers_file_arg = DeclareLaunchArgument(
        'controllers_file',
        default_value='mobile_manipulator_controllers.yaml',
        description='Controller configuration file for simulation launches',
    )
    namespace_arm_arg = DeclareLaunchArgument(
        'namespace_arm',
        default_value='',
        description='Namespace for the arm robot',
    )
    planner_backend_arg = DeclareLaunchArgument(
        'planner_backend',
        default_value='legacy',
        description='Planner backend to use: legacy or moveit',
        choices=['legacy', 'moveit'],
    )
    enable_wall_scene_sync_arg = DeclareLaunchArgument(
        'enable_wall_scene_sync',
        default_value='false',
        description='Enable wall-marker collision sync into MoveIt PlanningScene',
    )

    robot_ip = LaunchConfiguration('robot_ip')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    tf_prefix = LaunchConfiguration('tf_prefix')
    prefix = LaunchConfiguration('prefix')
    launch_rviz = LaunchConfiguration('launch_rviz')
    ur_type = LaunchConfiguration('ur_type')
    simulation = LaunchConfiguration('sim')
    hybrid_sim = LaunchConfiguration('hybrid_sim')
    mode = LaunchConfiguration('mode')
    moveit_mode = LaunchConfiguration('moveit_mode')
    moveit_use_sim_time = LaunchConfiguration('moveit_use_sim_time')
    controllers_file = LaunchConfiguration('controllers_file')
    namespace_arm = LaunchConfiguration('namespace_arm')
    planner_backend = LaunchConfiguration('planner_backend')
    enable_wall_scene_sync = LaunchConfiguration('enable_wall_scene_sync')
    stack_launch_rviz = PythonExpression(
        ["'false' if '", planner_backend, "' == 'moveit' else '", launch_rviz, "'"]
    )

    joint_controller_type = PythonExpression(
        ["'scaled_joint_trajectory_controller' if '", planner_backend, "' == 'moveit' else 'joint_trajectory_controller'"]
    )
    effective_moveit_mode = PythonExpression(
        ["'", moveit_mode, "' if '", moveit_mode, "' != 'auto' else '", mode, "'"]
    )
    effective_moveit_use_sim_time = PythonExpression(
        [
            "'",
            moveit_use_sim_time,
            "' if '",
            moveit_use_sim_time,
            "' != 'auto' else ('false' if '",
            simulation,
            "' == 'false' or '",
            hybrid_sim,
            "' == 'true' else 'true')",
        ]
    )
    inverted_sim = NotSubstitution(simulation)

    arm_group = GroupAction(
        [
            PushRosNamespace(namespace_arm),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(ur_control_launch),
                launch_arguments={
                    'ur_type': ur_type,
                    'robot_ip': robot_ip,
                    'use_fake_hardware': use_fake_hardware,
                    'tf_prefix': tf_prefix,
                    'prefix': prefix,
                    'mode': mode,
                    'sim': inverted_sim,
                    'stack_launch_rviz': stack_launch_rviz,
                    'initial_joint_controller': joint_controller_type,
                    'activate_joint_controller': 'true',
                }.items(),
                condition=IfCondition(hybrid_sim),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(ur_sim_control_launch),
                launch_arguments={
                    'ur_type': ur_type,
                    'tf_prefix': tf_prefix,
                    'prefix': prefix,
                    'mode': mode,
                    'stack_launch_rviz': stack_launch_rviz,
                    'controllers_file': controllers_file,
                }.items(),
                condition=IfCondition(AndSubstitution(simulation, NotSubstitution(hybrid_sim))),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(publisher_launch),
                launch_arguments={'check_starting_point': 'false'}.items(),
                condition=IfCondition(PythonExpression(["'", planner_backend, "' == 'legacy'"])),
            ),
            Node(
                package='arm_control',
                executable='planner_node',
                name='robotic_arm_planner_node',
                output='screen',
                condition=IfCondition(PythonExpression(["'", planner_backend, "' == 'legacy'"])),
            ),
            Node(
                package='arm_control',
                executable='end_effector_pose_node',
                name='end_effector_pose_node',
                output='screen',
            ),
            Node(
                package='arm_control',
                executable='position_sender_node',
                name='position_sender_node',
                output='screen',
                parameters=[{'planner_backend': planner_backend}],
            ),
        ]
    )

    moveit_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_launch),
        launch_arguments={
            'ur_type': ur_type,
            'mode': effective_moveit_mode,
            'namespace_arm': namespace_arm,
            'tf_prefix': tf_prefix,
            'use_fake_hardware': use_fake_hardware,
            'use_sim_time': effective_moveit_use_sim_time,
            'launch_rviz': launch_rviz,
        }.items(),
        condition=IfCondition(PythonExpression(["'", planner_backend, "' == 'moveit'"])),
    )

    moveit_planner_node = Node(
        package='arm_control',
        executable='moveit_planner_node',
        name='moveit_planner_node',
        output='screen',
        parameters=[
            {
                'group_name': 'arm_manipulator',
                'end_effector_link': 'arm_tool0',
                'planning_frame': 'arm_base',
                'mode': effective_moveit_mode,
                'use_sim_time': ParameterValue(effective_moveit_use_sim_time, value_type=bool),
                'enable_wall_scene_sync': ParameterValue(enable_wall_scene_sync, value_type=bool),
            }
        ],
        condition=IfCondition(PythonExpression(["'", planner_backend, "' == 'moveit'"])),
    )

    return LaunchDescription(
        [
            robot_ip_arg,
            use_fake_hardware_arg,
            tf_prefix_arg,
            prefix_arg,
            launch_rviz_arg,
            ur_type_arg,
            simulation_arg,
            hybrid_sim_arg,
            mode_arg,
            moveit_mode_arg,
            moveit_use_sim_time_arg,
            controllers_file_arg,
            namespace_arm_arg,
            planner_backend_arg,
            enable_wall_scene_sync_arg,
            arm_group,
            moveit_include,
            moveit_planner_node,
        ]
    )
