from setuptools import find_packages, setup
import glob
import os

package_name = 'inspector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/clearpath_standard_demo.launch.py',
            'launch/custom_nav2_launch.py',
            'launch/mola_husky_slam.launch.py',
            'launch/lidar_converter.launch.py',
            'launch/local_slam.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/nav2_params.yaml',
        ]),
        ('share/' + package_name + '/rviz', [
            'rviz/husky.rviz',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rugved Raote',
    maintainer_email='rugvedraote@gmail.com',
    description='Autonomous lidar robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'inspector = inspector.inspector:main',
            'timestamp_fixer = inspector.timestamp_fixer:main',
        ],
        'launch.launch_description': [
            'navigation = inspector.launch.navigation:generate_launch_description',
            'clearpath_standard_demo = inspector.launch.clearpath_standard_demo:generate_launch_description',
            'mola_husky_slam = inspector.launch.mola_husky_slam:generate_launch_description',
        ],
    },
)
