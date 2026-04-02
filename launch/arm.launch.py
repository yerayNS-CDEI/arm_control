from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
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


def launch_moveit_planner_node(context, *args, **kwargs):
    """Evaluate substitutions at launch time to set moveit_planner_node parameters."""
    moveit_mode = context.launch_configurations.get('moveit_mode', 'auto')
    mode = context.launch_configurations.get('mode', 'arm')
    moveit_use_sim_time = context.launch_configurations.get('moveit_use_sim_time', 'auto')
    moveit_planning_pipeline = context.launch_configurations.get(
        'moveit_planning_pipeline', 'pilz_industrial_motion_planner'
    )
    moveit_pose_planner_id = context.launch_configurations.get(
        'moveit_pose_planner_id', 'PTP'
    )
    moveit_joint_planner_id = context.launch_configurations.get(
        'moveit_joint_planner_id', 'PTP'
    )
    simulation = context.launch_configurations.get('sim', 'false')
    hybrid_sim = context.launch_configurations.get('hybrid_sim', 'false')
    enable_wall_scene_sync = context.launch_configurations.get('enable_wall_scene_sync', 'false')
    planner_backend = context.launch_configurations.get('planner_backend', 'legacy')
    
    # Only launch if planner_backend is 'moveit'
    if planner_backend != 'moveit':
        return []
    
    # Evaluate effective_moveit_mode: use moveit_mode if not 'auto', else use mode
    effective_mode = moveit_mode if moveit_mode != 'auto' else mode
        
    # Evaluate enable_base_collision: False if effective_mode is 'full', else True
    enable_base_collision = False if effective_mode == 'full' else True
    
    # Evaluate use_sim_time
    if moveit_use_sim_time != 'auto':
        use_sim_time = moveit_use_sim_time == 'true'
    else:
        use_sim_time = not (simulation == 'false' or hybrid_sim == 'true')
    
    # Convert enable_wall_scene_sync to bool
    wall_scene_sync = enable_wall_scene_sync == 'true'
    
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
                'mode': effective_mode,
                'use_sim_time': use_sim_time,
                'planning_pipeline': moveit_planning_pipeline,
                'pose_planner_id': moveit_pose_planner_id,
                'joint_planner_id': moveit_joint_planner_id,
                'enable_wall_scene_sync': wall_scene_sync,
                'enable_base_collision': enable_base_collision,
            }
        ],
    )
    
    return [moveit_planner_node]


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
    ethercat_interface_arg = DeclareLaunchArgument(
        'ethercat_interface',
        default_value='eno1',
        description='Network interface used by the Navi Wall EtherCAT master',
    )
    rviz_config_file_arg = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=PathJoinSubstitution(
            [FindPackageShare('arm_control'), 'rviz', 'moveit.rviz']
        ),
        description='RViz config file for the MoveIt RViz instance',
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
    moveit_planning_pipeline_arg = DeclareLaunchArgument(
        'moveit_planning_pipeline',
        default_value='pilz_industrial_motion_planner',
        description='MoveIt pipeline for the custom planner node (move_group or pilz_industrial_motion_planner)',
    )
    moveit_pose_planner_id_arg = DeclareLaunchArgument(
        'moveit_pose_planner_id',
        default_value='PTP',
        description='Planner id for pose goals in the custom MoveIt planner node',
    )
    moveit_joint_planner_id_arg = DeclareLaunchArgument(
        'moveit_joint_planner_id',
        default_value='PTP',
        description='Planner id for joint goals in the custom MoveIt planner node',
    )
    moveit_joint_states_topic_arg = DeclareLaunchArgument(
        'moveit_joint_states_topic',
        default_value='',
        description='Optional joint_states topic override for MoveIt',
    )
    moveit_controller_name_arg = DeclareLaunchArgument(
        'moveit_controller_name',
        default_value='joint_trajectory_controller',
        description=(
            'Trajectory controller used by the MoveIt/real-robot path. '
            'Defaults to joint_trajectory_controller because scaled_joint_trajectory_controller '
            'is currently crashing in the Humble UR driver on the real robot.'
        ),
        choices=['joint_trajectory_controller', 'scaled_joint_trajectory_controller'],
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
    ethercat_interface = LaunchConfiguration('ethercat_interface')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    ur_type = LaunchConfiguration('ur_type')
    simulation = LaunchConfiguration('sim')
    hybrid_sim = LaunchConfiguration('hybrid_sim')
    mode = LaunchConfiguration('mode')
    moveit_mode = LaunchConfiguration('moveit_mode')
    moveit_use_sim_time = LaunchConfiguration('moveit_use_sim_time')
    moveit_joint_states_topic = LaunchConfiguration('moveit_joint_states_topic')
    moveit_controller_name = LaunchConfiguration('moveit_controller_name')
    controllers_file = LaunchConfiguration('controllers_file')
    namespace_arm = LaunchConfiguration('namespace_arm')
    planner_backend = LaunchConfiguration('planner_backend')
    enable_wall_scene_sync = LaunchConfiguration('enable_wall_scene_sync')
    stack_launch_rviz = PythonExpression(
        ["'false' if '", planner_backend, "' == 'moveit' else '", launch_rviz, "'"]
    )

    joint_controller_type = PythonExpression(
        [
            "'",
            moveit_controller_name,
            "' if '",
            planner_backend,
            "' == 'moveit' else 'joint_trajectory_controller'",
        ]
    )
    # Determine the effective mode for MoveIt (used for moveit_include launch args)
    # - If moveit_mode is explicitly set (!= 'auto'), use it
    # - Otherwise, fall back to the general mode parameter
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
                    'ethercat_interface': ethercat_interface,
                    'stack_launch_rviz': stack_launch_rviz,
                    'initial_joint_controller': joint_controller_type,
                    'activate_joint_controller': 'true',
                }.items(),
                # condition=IfCondition(hybrid_sim),
            ),
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(ur_sim_control_launch),
            #     launch_arguments={
            #         'ur_type': ur_type,
            #         'tf_prefix': tf_prefix,
            #         'prefix': prefix,
            #         'mode': mode,
            #         'stack_launch_rviz': stack_launch_rviz,
            #         'controllers_file': controllers_file,
            #     }.items(),
            #     condition=IfCondition(AndSubstitution(simulation, NotSubstitution(hybrid_sim))),
            # ),
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
            'rviz_config_file': rviz_config_file,
            'joint_states_topic': moveit_joint_states_topic,
            'default_trajectory_controller': moveit_controller_name,
        }.items(),
        condition=IfCondition(PythonExpression(["'", planner_backend, "' == 'moveit'"])),
    )

    moveit_planner_node_opaque = OpaqueFunction(function=launch_moveit_planner_node)

    return LaunchDescription(
        [
            robot_ip_arg,
            use_fake_hardware_arg,
            tf_prefix_arg,
            prefix_arg,
            launch_rviz_arg,
            ethercat_interface_arg,
            rviz_config_file_arg,
            ur_type_arg,
            simulation_arg,
            hybrid_sim_arg,
            mode_arg,
            moveit_mode_arg,
            moveit_use_sim_time_arg,
            moveit_planning_pipeline_arg,
            moveit_pose_planner_id_arg,
            moveit_joint_planner_id_arg,
            moveit_joint_states_topic_arg,
            moveit_controller_name_arg,
            controllers_file_arg,
            namespace_arm_arg,
            planner_backend_arg,
            enable_wall_scene_sync_arg,
            arm_group,
            moveit_include,
            moveit_planner_node_opaque,
        ]
    )
