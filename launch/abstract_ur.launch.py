import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ur_type_arg = DeclareLaunchArgument(
        'ur_type',
        default_value='ur10e',
        description='Type of UR robot (ur3, ur3e, ur5, ur5e, ur10, ur10e, ur16e, ur20, ur30)'
    )

    tf_prefix_arg = DeclareLaunchArgument(
        'tf_prefix',
        default_value='collision_',
        description='TF prefix for collision robot frames'
    )

    main_tf_prefix_arg = DeclareLaunchArgument(
        'main_tf_prefix',
        default_value='arm_',
        description='TF prefix for main robot frames'
    )

    tf_prefix = LaunchConfiguration('tf_prefix')
    main_tf_prefix = LaunchConfiguration('main_tf_prefix')

    abstract_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('arm_control'), 'launch', 'abstract_robot.launch.py')
        ]),
        launch_arguments={
            'root_col': [tf_prefix, 'base_link'],
            'tip_col': [tf_prefix, 'tool0'],
            'root': [main_tf_prefix, 'base_link'],
            'tip': [main_tf_prefix, 'tool0'],
            'main_tf_prefix': main_tf_prefix,
        }.items(),
    )

    return LaunchDescription([
        ur_type_arg,
        tf_prefix_arg,
        main_tf_prefix_arg,
        abstract_robot_launch,
    ])
