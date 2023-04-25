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
from typing import List

def get_robot_names(robots) -> List[str]:
    robot_name_list = []
    for instance in range(len(robots)):
        
        robot = robots[instance]
        robot_name_list.append(robot['name'])
    return robot_name_list

def generate_launch_description():

    # Launch description
    ld = LaunchDescription()

    # Launch description
    config = os.path.join(
        get_package_share_directory('sa_world_model'),
        'config',
        'sim.yaml'
        )

    ld.add_action(DeclareLaunchArgument('param_file', default_value=os.path.join(
        get_package_share_directory('sa_world_model'), 'config', 'robots.yaml')))

    
    robot_file = open(os.path.join(get_package_share_directory('sa_world_model'),'config','robots.yaml'))
    robots = yaml.safe_load(robot_file)
    robot_list, sensor_list = [], []
    for robot in robots:
        robot_list.append(robot['name'])
        sensor_list.append(robot['sensor']['type'])
    print(robot_list)

    # add the true_observation node
    ld.add_action(Node(
            package = 'sa_world_model',
            executable='true_objects_node',
            name= 'spawn_robot',
            output='both',
            # namespace= robot['name'],
            parameters=[config],
        ))

    # add robots and sensors based in config/robots.yaml
    for instance in range(len(robots)):
        
        robot = robots[instance]
        spawn_robot = Node(
            package = 'sa_world_model',
            executable='robot_node',
            name= 'sa_robot_node',
            output='both',
            namespace= robot['name'],
            parameters=[
                # LC('param_file'),
                {'name': robot['name']},
                {'type': robot['type']},
                {'x':    robot['sim_start']['x']},
                {'y':    robot['sim_start']['y']},
                {'yaw':  robot['sim_start']['yaw']},
                {'update_frequency': robot['update_frequency']},
                {'fov_x_list': robot['sensor']['fov']['x']},
                {'fov_y_list': robot['sensor']['fov']['y']},
                {'robot_id': robot['domain_id']},
            ],
        )
        
        ld.add_action(spawn_robot)
    
        # sensor node
        spawn_sensor = Node(
            package = 'sa_world_model',
            executable='sensor_node',
            name= 'spawn_sensor',
            output='both',
            namespace= robot['name'],
            parameters=[
                {'x':    robot['sim_start']['x']},
                {'y':    robot['sim_start']['y']},
                {'yaw':  robot['sim_start']['yaw']},
                {'sensor_type': robot['sensor']['type']},
                {'robot_type': robot['type']},
                {'fov_x_list': robot['sensor']['fov']['x']},
                {'fov_y_list': robot['sensor']['fov']['y']},
                {'detection_frequency': robot['sensor']['detection_frequency']},
                {'robot_id': robot['domain_id']},
                {'alpha': robot['sensor']['alpha']},
                {'r0': robot['sensor']['r0']},
            ],
        )
        ld.add_action(spawn_sensor)
        
        # plan node
        if robot['type'] == 'uav':
            spawn_planner = Node(
                package = 'sa_sensor_planner',
                executable='sensor_planner_node',
                name= 'sensor_planner_node',
                output='both',
                namespace= robot['name'],
                parameters=[
                    config,
                    {'robot_id': robot['domain_id']},
                    {'v_max': robot['v_max']},
                    {'robot_name_list': robot_list},
                ],
            )
            ld.add_action(spawn_planner)

            # dummy_sml = Node(
            #     package = 'sa_sensor_planner',
            #     executable='dummy_sml_node',
            #     name= 'dummy_sml_node',
            #     output='both',
            #     namespace= robot['name'],
            # )
            # ld.add_action(dummy_sml)
    
        
    return ld
