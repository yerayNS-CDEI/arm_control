import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    ur_type_arg = DeclareLaunchArgument(
        'ur_type',
        default_value='ur10e',
        description='Type of UR robot (ur3, ur3e, ur5, ur5e, ur10, ur10e, ur16e, ur20, ur30)'
    )

    # Mode argument: determines if using arm-only or full mobile manipulator
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='full',
        description='Collision robot mode: arm (arm-only) or full (mobile manipulator with base)',
        choices=['arm', 'full']
    )

    # For collision checking, root and tip depend on mode:
    # - arm mode: arm_base_link -> arm_tool0 (arm only URDF, arm_ tf_prefix, 6 DOF)
    # - full mode: base_footprint -> arm_tool0 (mobile manipulator URDF, arm has arm_ prefix, 8 DOF)
    root_col_arg = DeclareLaunchArgument(
        'root_col',
        default_value=PythonExpression(["'base_footprint' if '", LaunchConfiguration('mode'), "' == 'full' else 'arm_base_link'"]),
        description='Base frame for collision checking'
    )

    tip_col_arg = DeclareLaunchArgument(
        'tip_col',
        default_value='arm_tool0',
        description='End-effector frame for collision checking'
    )

    main_tf_prefix_arg = DeclareLaunchArgument(
        'main_tf_prefix',
        default_value='arm_',
        description='TF prefix for main robot frames (used for startup warm-up)'
    )

    main_tf_prefix = LaunchConfiguration('main_tf_prefix')
    root_col = LaunchConfiguration('root_col')
    tip_col = LaunchConfiguration('tip_col')
    ur_type = LaunchConfiguration('ur_type')
    mode = LaunchConfiguration('mode')

    # Include collision_view_ur.launch.py to provide robot_state_publisher for collision namespace
    collision_view_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('arm_control'), 'launch', 'collision_view_ur.launch.py')
        ]),
        launch_arguments={
            'ur_type': ur_type,
            'tf_prefix': 'arm_',
            'mode': mode,
            'launch_rviz': 'false',
        }.items(),
    )

    abstract_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('arm_control'), 'launch', 'abstract_robot.launch.py')
        ]),
        launch_arguments={
            'root_col': root_col,
            'tip_col': tip_col,
            'root': [main_tf_prefix, 'base_link'],
            'tip': [main_tf_prefix, 'tool0'],
            'main_tf_prefix': main_tf_prefix,
            'mode': mode,
        }.items(),
    )

    return LaunchDescription([
        ur_type_arg,
        mode_arg,
        root_col_arg,
        tip_col_arg,
        main_tf_prefix_arg,
        collision_view_launch,
        abstract_robot_launch,
    ])
