#!/usr/bin/env python3

from time import sleep
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import numpy as np
from typing import List
from std_msgs.msg import Float32
from sa_msgs.msg import RobotState, Detection, DetectionArray
from sa_world_model.util.sensor import sensor
from sa_world_model.util.camera_sensor import camera
from sa_world_model.util.beam_sensor import beam

class sensor_node(Node):
    def __init__(self):
        super().__init__('sa_sensor_model')

        self.declare_parameter('x', 10.0)
        self.declare_parameter('y', 10.0)
        self.declare_parameter('yaw', 0.0)

        self.declare_parameter('sensor_type', 'static')
        self.declare_parameter('robot_type', 'static')
        self.declare_parameter('fov_x_list', [0.0])
        self.declare_parameter('fov_y_list', [0.0])

        self.declare_parameter('detection_frequency', 10.0)
        self.declare_parameter('robot_id', 1)

        self.declare_parameter('alpha', 1.0)
        self.declare_parameter('r0', 5.0)
        
        # Attribute ===============================================
        x = self.get_parameter('x').get_parameter_value().double_value
        y = self.get_parameter('y').get_parameter_value().double_value
        yaw = self.get_parameter('yaw').get_parameter_value().double_value
        
        self.sensor_type = self.get_parameter('sensor_type').get_parameter_value().string_value
        robot_type = self.get_parameter('robot_type').get_parameter_value().string_value
        self.mobility = robot_type == 'uav'
        fov_x_list = self.get_parameter('fov_x_list').get_parameter_value().double_array_value
        fov_y_list = self.get_parameter('fov_y_list').get_parameter_value().double_array_value
        
        self.detection_frequency = self.get_parameter('detection_frequency').get_parameter_value().double_value

        alpha = self.get_parameter('alpha').get_parameter_value().double_value
        r0 = self.get_parameter('r0').get_parameter_value().double_value        

        self.robot_id = self.get_parameter('robot_id').get_parameter_value().integer_value
        self.robot_state = RobotState()
        self.robot_state.pose.x = x
        self.robot_state.pose.y = y
        self.robot_state.pose.yaw = yaw
        self.robot_state.robot_id = self.robot_id
        self.get_logger().info("init sensor with type %s" % self.sensor_type)
        if self.sensor_type == 'camera':
            self.sensor = camera(fov_x_list, fov_y_list, self.mobility, 
                                self.robot_id, alpha, r0, self.robot_state.pose)
        elif self.sensor_type == 'beam':
            self.sensor = beam(fov_x_list, fov_y_list, self.mobility, self.robot_id)
        else:
            self.sensor = sensor(fov_x_list, fov_y_list, self.mobility, self.robot_id)
    
        # publishers =============
        # self.get_logger().info("node pub is {}".format(self.get_namespace() + '/' + self.sensor_type + '/detection'))
        # self.create_timer(self.detection_frequency, self.true_objects_callback)
        self.noisy_objects_pub_ = self.create_publisher(
            DetectionArray, 
            self.get_namespace() + '/' + self.sensor_type + '/detection', 
            10)

        # subscribers ==============
        self.robot_state_sub_ = self.create_subscription(
            RobotState,
            self.get_namespace() + '/state',
            self.robot_state_callback,
            10,
        )

        self.true_object_sub_ = self.create_subscription(            
            DetectionArray, 
            '/sa_world_model/true_objects_state',
            self.sensor_model_callback,
            10,
        )
        

    def robot_state_callback(self, msg: RobotState):
        self.robot_state = msg
    
    def sensor_model_callback(self, msg: DetectionArray):
        # print("msg recevied to pub in {}".format(self.get_namespace() + '/' + self.type + '/detection'))
        # if self.robot_state.robot_id != self.robot_id:
        #     return
        new_msg = self.sensor.sensor_observation(msg, self.robot_state.pose)
        
        if self.sensor_type == 'beam' and not new_msg.beam_detection:
            return

        if self.robot_state.robot_id != 1 and len(new_msg.detections) == 0:
            return
        new_msg.detection_time = msg.detection_time
        new_msg.robot_state = self.robot_state
        new_msg.sensor_type = self.sensor_type
        self.noisy_objects_pub_.publish(new_msg)
    


def main(args=None):
    rclpy.init(args=args)
    node = sensor_node()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()