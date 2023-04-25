#!/usr/bin/env python3

from time import sleep
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import numpy as np
from typing import List, Dict
from std_msgs.msg import Float32
from sa_msgs.msg import RobotState, Detection, DetectionArray, EstimationMsg, PoseEstimation, Velocity
from sa_trajectory_estimation.util.jpda import jpda
import json, os
from ament_index_python.packages        import get_package_share_directory

class trajectory_estimation_node(Node):

    def __init__(self):
        super().__init__('trajectory_estimation_node')

        self._load_sensor_para()

        self.declare_parameter('robot_list', ['robot'])
        self.declare_parameter('sensor_list', ['sensor'])
        self.declare_parameter('detection_frequency', 10.0)
        self.declare_parameter('static_object_list', [])
        self.declare_parameter('dynamic_object_list', ['car'])

        robot_list = self.get_parameter('robot_list').get_parameter_value().string_array_value
        sensor_list = self.get_parameter('sensor_list').get_parameter_value().string_array_value
        self.dynamic_object_list = self.get_parameter('dynamic_object_list').get_parameter_value().string_array_value
        self.static_object_list = self.get_parameter('static_object_list').get_parameter_value().string_array_value
        detection_frequency = self.get_parameter('detection_frequency').get_parameter_value().double_value
        # Attribute ===============================================
        
        self.t = -0.01
        self.dt = 1 / detection_frequency
        self._init_tracker()
    
        # publishers =============
        # self.get_logger().info("node pub is {}".format(self.get_namespace() + '/' + self.type_ + '/detection'))
        # self.create_timer(self.detection_frequency, self.true_objects_callback)
        self.trajectory_estimation_pub_ = self.create_publisher(
            EstimationMsg, 
            '/sa_tajectory_estimation/estimation', 
            10)
        

        # subscribers ==============

        for robot_name, sensor_type in zip(robot_list, sensor_list):

            topic = '/' + robot_name + '/' + sensor_type + '/detection'
            self.get_logger().info("sub to {}".format(topic))
            self.create_subscription(
                DetectionArray,
                topic,
                self.trajectory_estimation_callback,
                10,
            )

    def _load_sensor_para(self):
        path_ = os.path.join(get_package_share_directory('sa_trajectory_estimation'),'config','sensor.json')    
        with open(path_) as json_file:
                
            self.sensor_para_list = json.load(json_file)
    
    def _init_tracker(self):

        self.tracker: Dict[str, jpda] = dict()

        for static_label in self.static_object_list:
            a_tracker = jpda(self.dt, 
                self.sensor_para_list["sensors"],
                ConfirmationThreshold = [0, 1],
                DeletionThreshold = [5, 5], 
                IsStatic = True,
                label=static_label)
            self.tracker[static_label] = a_tracker            

        for dynamic_label in self.dynamic_object_list:

            # this can be improved by specifying every label's semantic
            a_tracker = jpda(self.dt, 
                self.sensor_para_list["sensors"], 
                ConfirmationThreshold = self.sensor_para_list["ConfirmationThreshold"],
                DeletionThreshold = self.sensor_para_list["DeletionThreshold"], 
                IsStatic = False,
                label=dynamic_label)
            self.tracker[dynamic_label] = a_tracker
        
    def _classify_observations(self, msg: DetectionArray) -> Dict[str, List[List[float]]]:

        sorted_obs_dict = {}
        for object_label in self.static_object_list + self.dynamic_object_list:
            sorted_obs_dict[object_label] = list()

        for detection_ in msg.detections:
            
            assert detection_.detection_label in self.static_object_list + self.dynamic_object_list, \
                RuntimeError("detection label {} strange".format(detection_.detection_label))
            z = [detection_.pose.x, detection_.pose.y]
            sorted_obs_dict[detection_.detection_label].append(z)
        
        return sorted_obs_dict

    def trajectory_estimation_callback(self, msg: DetectionArray):

        est_msg = EstimationMsg()
        est_msg.robot_state = msg.robot_state
        est_msg.detection_time = msg.detection_time
        if msg.robot_state.robot_id == 0:
            self.get_logger().info("sub to {}".format(msg))

        xs = [msg.robot_state.pose.x, msg.robot_state.pose.y, msg.robot_state.pose.yaw]

        if msg.sensor_type == 'beam':
            # if receive beam sensor data, pass it without updating filter state
            est_msg.discrete_detections = msg.discrete_detections
        else:

            if self.t <= 0:
                dt = self.dt
            else:
                dt = msg.detection_time - self.t
            self.t = msg.detection_time
            # process the sensor data in jpda
            # for i, detection_ in enumerate(msg.detections):
            #     traj_ = PoseEstimation()
            #     traj_.pose = detection_.pose
            #     traj_.detection_label = detection_.detection_label
            #     traj_.trajectory_id = i + 1
            #     traj_.covariance = [0.0] * 16
            #     traj_.velocity = Velocity()
            #     est_msg.pose_estimations.append(traj_)
            
            sorted_obs_dict = self._classify_observations(msg)
            self.get_logger().info("%s" % sorted_obs_dict)
            for label in self.static_object_list:
                # static object update
                z_k = sorted_obs_dict[label]
                self.tracker[label].track_update(
                    t = self.t, 
                    z_k=z_k, 
                    dt=dt, 
                    xs = xs, 
                    robot_id = msg.robot_state.robot_id,
                    est_msg=est_msg,
                )

            for label in self.dynamic_object_list:
                # static object update
                z_k = sorted_obs_dict[label]
                self.tracker[label].track_update(
                    t = self.t, 
                    z_k=z_k, 
                    dt=dt, 
                    xs = xs, 
                    robot_id = msg.robot_state.robot_id,
                    est_msg=est_msg,
                )
            # prepare msg TODO
        self.trajectory_estimation_pub_.publish(est_msg)

        self.get_logger().info("node published %s" % len(est_msg.pose_estimations))

def main(args=None):
    rclpy.init(args=args)
    node = trajectory_estimation_node()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()