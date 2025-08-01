from setuptools import find_packages, setup
from glob import glob

package_name = 'robotic_arm_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/resource', glob('resource/*.pkl')),
        (f'share/{package_name}/resource', glob('resource/*.npy')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yeray',
    maintainer_email='yeray.navarro@estudiantat.upc.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'planner_node = robotic_arm_planner.planner_node:main',
            'base_placement_node = robotic_arm_planner.base_placement_node:main',
            'wall_discretization_node = robotic_arm_planner.wall_discretization_node:main',
            'wall_visualizer_node = robotic_arm_planner.visualize_wall_discretization_client:main'
        ],
    },
)
