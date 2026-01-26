from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'arm_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yeray Navarro',
    maintainer_email='yeray.navarro@upc.edu',
    description='Unified package combining ur_arm_control, robotic_arm_planner, and oliwall_sensors',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Control nodes
            'end_effector_pose_node = control.end_effector_pose_node:main',
            'example_move = control.example_move:main',
            'exhaustive_scan_node = control.exhaustive_scan_node:main',
            'freedrive_control = control.freedrive_control:main',
            'position_sender_node = control.position_sender_node:main',
            'publisher_joint_trajectory_planned = control.publisher_joint_trajectory_planned:main',
            'remote_DASHBOARD = control.remote_DASHBOARD:main',
            'remote_SCRIPT = control.remote_SCRIPT:main',
            'robot_command_logger = control.robot_command_logger:main',
            'robot_status_check = control.robot_status_check:main',
            'send_and_monitor_trajectory = control.send_and_monitor_trajectory:main',
            'UI = control.UI:main',
            
            # Planner nodes
            'base_placement_node = planner.base_placement_node:main',
            'planner_node = planner.planner_node:main',
            'visualize_wall_discretization_client = planner.visualize_wall_discretization_client:main',
            'wall_discretization_node = planner.wall_discretization_node:main',
            
            # Sensor nodes
            'hyperspectral_node = sensors.hyperspectral_node:main',
            'sensors_distance = sensors.sensors_distance:main',
            'sensors_distance_orientation = sensors.sensors_distance_orientation:main',
            'sensors_distance_orientation_sim = sensors.sensors_distance_orientation_sim:main',
            'align_ee_to_wall = sensors.align_ee_to_wall:main',
            'arduino_sensors = sensors.arduino_sensors:main',
            'test_hyperspectral_client = sensors.test_hyperspectral_client:main',
            'arduino_sensors_sim = sensors.arduino_sensors_sim:main',
        ],
    },
)
