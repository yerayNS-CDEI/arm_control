import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments for node parameters
    root_col_arg = DeclareLaunchArgument(
        'root_col',
        default_value='base_link',
        description='Base frame of the collision robot'
    )
    tip_col_arg = DeclareLaunchArgument(
        'tip_col',
        default_value='ee_link',
        description='End-effector link of the collision robot'
    )
    root_arg = DeclareLaunchArgument(
        'root',
        default_value='base_link',
        description='Base frame of the main robot'
    )
    tip_arg = DeclareLaunchArgument(
        'tip',
        default_value='ee_link',
        description='End-effector link of the main robot'
    )

    root = LaunchConfiguration('root')
    tip = LaunchConfiguration('tip')
    root_col = LaunchConfiguration('root_col')
    tip_col = LaunchConfiguration('tip_col')

    constrained_manip_params = {
        'root': root,
        'tip': tip,
    }
    path_collision_params = {
        'root': root_col,
        'tip': tip_col,
    }

    constrained_manip_node = Node(
        package='arm_control',
        executable='constrained_manipulability_node_mod',
        name='constrained_manipulability_node_mod',
        parameters=[constrained_manip_params],
        arguments=['--ros-args', '--log-level', 'INFO'],
    )

    path_collision_node = Node(
        package='arm_control',
        executable='path_collision_checking',
        name='path_collision_checking',
        namespace='collision',
        parameters=[path_collision_params],
        arguments=['--ros-args', '--log-level', 'INFO'],
    )

    return LaunchDescription([
        root_col_arg,
        tip_col_arg,
        root_arg,
        tip_arg,
        constrained_manip_node,
        path_collision_node,
    ])
