import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'sa_world_model'


# https://github.com/ros2/tutorials/blob/master/rclpy_tutorials/setup.py


setup(
    name=package_name,
    version='0.0.0',
    # packages=[package_name],
    packages = find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')), 
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='agc2',
    maintainer_email='xmcx731@tamu.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_node = sa_world_model.sa_sensor_node:main',
            'robot_node = sa_world_model.sa_robot_node:main',
            'true_objects_node = sa_world_model.sa_true_objects:main',
        ],
    },
)
