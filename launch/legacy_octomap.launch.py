from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='full',
        description='Robot mode used to select the base frame for optional ground filtering.',
        choices=['arm', 'full'],
    )
    point_cloud_topic_arg = DeclareLaunchArgument(
        'point_cloud_topic',
        default_value='/combined_cloud_filtered',
        description='Robot-free fused PointCloud2 topic consumed by octomap_server.',
    )
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='odom',
        description='Fixed frame used by octomap_server for published maps.',
    )
    resolution_arg = DeclareLaunchArgument(
        'resolution',
        default_value='0.05',
        description='Octomap voxel resolution in meters.',
    )
    max_range_arg = DeclareLaunchArgument(
        'max_range',
        default_value='3.0',
        description='Maximum sensor range integrated into the octomap.',
    )

    mode = LaunchConfiguration('mode')
    point_cloud_topic = LaunchConfiguration('point_cloud_topic')
    frame_id = LaunchConfiguration('frame_id')
    resolution = LaunchConfiguration('resolution')
    max_range = LaunchConfiguration('max_range')

    base_frame_id = PythonExpression([
        "'base_footprint' if '", mode, "' == 'full' else 'arm_base_link'",
    ])

    octomap_server_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[
            {
                'frame_id': frame_id,
                'base_frame_id': base_frame_id,
                'resolution': resolution,
                'sensor_model.max_range': max_range,
                'filter_ground_plane': True,
                'compress_map': True,
                'publish_free_space': False,
                'latch': True,
            }
        ],
        remappings=[
            ('cloud_in', point_cloud_topic),
            ('octomap_binary', '/octomap_filtered'),
        ],
    )

    return LaunchDescription([
        mode_arg,
        point_cloud_topic_arg,
        frame_id_arg,
        resolution_arg,
        max_range_arg,
        octomap_server_node,
    ])