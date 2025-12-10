from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    urdf_path = os.path.join(
        get_package_share_directory('ur_arm_control'),
        'urdf',
        'BIM_model.urdf'
        # 'ur10e.urdf'
    )
    return LaunchDescription([
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            arguments=[urdf_path]
        ),
        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     name='robot_state_publisher',
        #     parameters=[{'robot_description': open(urdf_path).read()}],
        #     output='screen'
        # ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(
                get_package_share_directory('ur_arm_control'),
                'rviz',
                'view_urdf.rviz'
            )],
            output='screen',
        )
    ])
