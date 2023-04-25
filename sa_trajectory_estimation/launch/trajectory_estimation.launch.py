#!/usr/bin/env python3

import os
import launch
import xacro
import yaml

from ament_index_python.packages        import get_package_share_directory
from launch                             import LaunchDescription
from launch.actions                     import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources  import PythonLaunchDescriptionSource
from launch.substitutions               import LaunchConfiguration as LC
from launch_ros.actions                 import Node
from launch.conditions                  import IfCondition
   

def generate_launch_description():

    # Launch description
    ld = LaunchDescription()
    
    robot_file = open(os.path.join(get_package_share_directory('sa_world_model'),'config','robots.yaml'))
    robots = yaml.safe_load(robot_file)

    config = os.path.join(
        get_package_share_directory('sa_world_model'),
        'config',
        'sim.yaml'
        )

    robot_list, sensor_list = [], []
    for robot in robots:
        robot_list.append(robot['name'])
        sensor_list.append(robot['sensor']['type'])

    # add the true_observation node
    ld.add_action(Node(
        package = 'sa_trajectory_estimation',
        executable='trajectory_estimation_node',
        name= 'trajectory_estimation_node',
        output='both',
        # namespace='sa_trajectory_estimation',
        parameters=[
            config,
            {'robot_list': robot_list},
            {'sensor_list': sensor_list},
            {'detection_frequency': robot['sensor']['detection_frequency']},
        ],
    ))
    
        
    return ld
