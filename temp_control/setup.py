from setuptools import find_packages, setup
import glob

package_name = 'ur_arm_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*.py')),
        ('share/' + package_name + '/config', glob.glob('config/*.yaml')),
        ('share/' + package_name + '/rviz', glob.glob('rviz/*.rviz')),
        ('share/' + package_name + '/urdf', glob.glob('urdf/*')), 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yeray',
    maintainer_email='yeray.navarro@upc.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'end_effector_pose_node = ur_arm_control.end_effector_pose_node:main',
            'example_move = ur_arm_control.example_move:main',
            'publisher_joint_trajectory_controller = ur_arm_control.publisher_joint_trajectory_controller:main',
            'publisher_joint_trajectory_planned = ur_arm_control.publisher_joint_trajectory_planned:main',
            'publisher_joint_trajectory_planned_topic = ur_arm_control.publisher_joint_trajectory_planned_topic:main',
            'send_and_monitor_trajectory = ur_arm_control.send_and_monitor_trajectory:main',
            'sensors_distance_orientation = ur_arm_control.sensors_distance_orientation:main',
            'sensors_distance = ur_arm_control.sensors_distance:main',
            'sensors_orientation = ur_arm_control.sensors_orientation:main',
            'sensors_arduino = ur_arm_control.sensors_arduino:main',
            'sensors_distance_orientation_sim = ur_arm_control.sensors_distance_orientation_sim:main',
        ],
    },
)
