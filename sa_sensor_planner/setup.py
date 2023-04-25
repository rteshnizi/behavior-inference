import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'sa_sensor_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'sensor_planner_node = sa_sensor_planner.sensor_planner_node:main',
            'dummy_sml_node = sa_sensor_planner.dummy_sml_node:main',
        ],
    },
)
