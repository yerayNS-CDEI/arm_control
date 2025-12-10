from setuptools import setup, find_packages

package_name = 'oliwall_sensors'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml'])
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yeray',
    maintainer_email='yeray.navarro@estudiantat.upc.edu',
    description='Hyperspectral ROS2 interface',
    license='TODO',

    entry_points={
        'console_scripts': [
            'hyperspectral_node = oliwall_sensors.hyperspectral_node:main',
            'test_hyperspectral_client = oliwall_sensors.test_hyperspectral_client:main',
        ],
    },
)
