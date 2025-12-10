from setuptools import setup
import os
from glob import glob

package_name = 'arm_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=[
        package_name,
        f'{package_name}.control',
        f'{package_name}.planner',
        f'{package_name}.planner.planner_lib',
        f'{package_name}.sensors',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (os.path.join('share', package_name, 'resource'), glob('resource/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yeray Navarro',
    maintainer_email='yeray.navarro@upc.edu',
    description='Unified package combining ur_arm_control, robotic_arm_planner, robotic_arm_planner_interfaces, and oliwall_sensors',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Control nodes
            'end_effector_pose_node = arm_control.control.end_effector_pose_node:main',
            'example_move = arm_control.control.example_move:main',
            'publisher_joint_trajectory_controller = arm_control.control.publisher_joint_trajectory_controller:main',
            'publisher_joint_trajectory_planned = arm_control.control.publisher_joint_trajectory_planned:main',
            'publisher_joint_trajectory_planned_topic = arm_control.control.publisher_joint_trajectory_planned_topic:main',
            'remote_DASHBOARD = arm_control.control.remote_DASHBOARD:main',
            'remote_SCRIPT = arm_control.control.remote_SCRIPT:main',
            'send_and_monitor_trajectory = arm_control.control.send_and_monitor_trajectory:main',
            'sensors_distance = arm_control.control.sensors_distance:main',
            'sensors_distance_orientation = arm_control.control.sensors_distance_orientation:main',
            'sensors_distance_orientation_sim = arm_control.control.sensors_distance_orientation_sim:main',
            'sensors_orientation = arm_control.control.sensors_orientation:main',
            
            # Planner nodes
            'base_placement_node = arm_control.planner.base_placement_node:main',
            'planner_node = arm_control.planner.planner_node:main',
            'wall_discretization_node = arm_control.planner.wall_discretization_node:main',
            'visualize_wall_discretization_client = arm_control.planner.visualize_wall_discretization_client:main',
            
            # Sensor nodes
            'hyperspectral_node = arm_control.sensors.hyperspectral_node:main',
            'lenz_client = arm_control.sensors.lenz_client:main',
            'test_hyperspectral_client = arm_control.sensors.test_hyperspectral_client:main',
            'vis_nir_example = arm_control.sensors.vis_nir_example:main',
        ],
    },
)
