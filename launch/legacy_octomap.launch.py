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
        default_value='0.1',
        description='Octomap voxel resolution in meters.',
    )
    max_range_arg = DeclareLaunchArgument(
        'max_range',
        default_value='3.0',
        description='Maximum sensor range integrated into the octomap.',
    )
    point_cloud_min_z_arg = DeclareLaunchArgument(
        'point_cloud_min_z',
        default_value='-0.02',
        description='Discard incoming cloud points below this height in the map frame.',
    )
    point_cloud_max_z_arg = DeclareLaunchArgument(
        'point_cloud_max_z',
        default_value='2.5',
        description='Discard incoming cloud points above this height in the map frame.',
    )
    occupancy_min_z_arg = DeclareLaunchArgument(
        'occupancy_min_z',
        default_value='0.0',
        description='Do not create occupied octomap cells below this height.',
    )
    occupancy_max_z_arg = DeclareLaunchArgument(
        'occupancy_max_z',
        default_value='2.5',
        description='Do not create occupied octomap cells above this height.',
    )
    filter_speckles_arg = DeclareLaunchArgument(
        'filter_speckles',
        default_value='true',
        description='Drop isolated occupied voxels that usually come from sensor noise.',
        choices=['true', 'false'],
    )

    mode = LaunchConfiguration('mode')
    point_cloud_topic = LaunchConfiguration('point_cloud_topic')
    frame_id = LaunchConfiguration('frame_id')
    resolution = LaunchConfiguration('resolution')
    max_range = LaunchConfiguration('max_range')
    point_cloud_min_z = LaunchConfiguration('point_cloud_min_z')
    point_cloud_max_z = LaunchConfiguration('point_cloud_max_z')
    occupancy_min_z = LaunchConfiguration('occupancy_min_z')
    occupancy_max_z = LaunchConfiguration('occupancy_max_z')
    filter_speckles = LaunchConfiguration('filter_speckles')

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
                'pointcloud_min_z': point_cloud_min_z,
                'pointcloud_max_z': point_cloud_max_z,
                'occupancy_min_z': occupancy_min_z,
                'occupancy_max_z': occupancy_max_z,
                'filter_speckles': filter_speckles,
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
        point_cloud_min_z_arg,
        point_cloud_max_z_arg,
        occupancy_min_z_arg,
        occupancy_max_z_arg,
        filter_speckles_arg,
        octomap_server_node,
    ])