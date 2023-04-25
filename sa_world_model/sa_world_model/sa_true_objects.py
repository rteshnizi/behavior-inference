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
from sa_world_model.util.observation_generater import linear_traj, demo_traj
from sa_world_model.util.sensor import sensor

class true_objects(Node):
    def __init__(self):
        super().__init__('sa_true_objects')

        self.declare_parameter('detection_frequency', 10.0)

        self.detection_frequency = self.get_parameter('detection_frequency').get_parameter_value().double_value
        
        # Attribute ===============================================
    
        # publishers
        self.create_timer(1 / self.detection_frequency, self.true_objects_callback)
        self.true_objects_pub_ = self.create_publisher(
            DetectionArray, 
            'sa_world_model/true_objects_state',
            10)
        

        self.init_time = self.get_clock().now()
        self.pre_time = self.get_clock().now()
    
    def true_objects_callback(self):
        # publish true object positions

        msg = DetectionArray()
        cur_time = self.get_clock().now()
        # time implementation
        # https://answers.ros.org/question/321536/replacement-for-rospytimenow-in-ros2/
        duration: rclpy.duration.Duration = cur_time - self.init_time
        dt = duration.nanoseconds * 1e-9
        msg.detection_time = dt

        # pos = linear_traj(dt)
        # # print(pos)
        # a_detection_ = Detection()
        # a_detection_.detection_label = 'car'
        # a_detection_.pose.x = pos[0]
        # a_detection_.pose.y = pos[1]
        # msg.detections.append(a_detection_)

        pos_list = demo_traj(dt)

        for pos in pos_list:
            a_detection_ = Detection()
            a_detection_.detection_label = pos[-1]
            a_detection_.pose.x = pos[0]
            a_detection_.pose.y = pos[1]
            msg.detections.append(a_detection_)
    
        # msg.data = self.get_clock().now().to_msg()
        # print(msg.data)
        self.true_objects_pub_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = true_objects()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()