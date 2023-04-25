#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from sa_msgs.msg import RobotState, Plan, Fov, Pose
from sa_world_model.util.robot import robot
from sa_world_model.util.uav import uav

class robot_node(Node):
    def __init__(self):
        super().__init__('sa_robot_node')

        self.declare_parameter('x', 10.0)
        self.declare_parameter('y', 10.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('name', 'robot')
        self.declare_parameter('type', 'static')
        self.declare_parameter('update_frequency', 100.0)
        self.declare_parameter('fov_x_list', [0.0])
        self.declare_parameter('fov_y_list', [0.0])
        self.declare_parameter('robot_id', 1)
        self.declare_parameter('plan_frequency', 1.0)

        x = self.get_parameter('x').get_parameter_value().double_value
        y = self.get_parameter('y').get_parameter_value().double_value
        yaw = self.get_parameter('yaw').get_parameter_value().double_value
        self.name = self.get_parameter('name').get_parameter_value().string_value
        self.type_ = self.get_parameter('type').get_parameter_value().string_value
        update_frequency = self.get_parameter('update_frequency').get_parameter_value().double_value
        fov_x_list = self.get_parameter('fov_x_list').get_parameter_value().double_array_value
        fov_y_list = self.get_parameter('fov_y_list').get_parameter_value().double_array_value
        self.robot_id = self.get_parameter('robot_id').get_parameter_value().integer_value
        plan_frequency = self.get_parameter('plan_frequency').get_parameter_value().double_value
        self.plan_dt = 1 / plan_frequency
        self.action_mode = False
        
        # publishers
        self.create_timer(1 / update_frequency, self.state_pub_callback)

        self.state_pub_ = self.create_publisher(
            RobotState, 
            self.get_namespace() + '/state',
            10)

        # receivers
        if self.type_ != 'static':
            # listen to planner
            self.plan_sub_ = self.create_subscription(
                Plan,
                self.get_namespace() + '/plan',
                self._plan_callback,
                10,
            )
            self.robot = uav(x, y, yaw)
        else:
            self.robot = robot(x, y, yaw)
            self.plan_sub_ = None
        
        self.time = self.get_clock().now()
        self.fov = Fov()

        for x, y in zip(fov_x_list, fov_y_list):
            corner = Pose()
            corner.x = x
            corner.y = y
            self.fov.corners.append(corner)
        self.plan_start_time = self.get_clock().now()
        self.plan = Plan()
        
    
    def state_pub_callback(self):
        # publish true object positions

        cur_time = self.get_clock().now()
        # time implementation
        # https://answers.ros.org/question/321536/replacement-for-rospytimenow-in-ros2/
        duration: rclpy.duration.Duration = cur_time - self.time
        dt = duration.nanoseconds * 1e-9

        self.time = cur_time

        self._check_plan_update()

        self.robot._dynamics(dt)

        msg = RobotState()

        msg.pose = self.robot.pose
        msg.robot_id = self.robot_id
        msg.fov = self.fov
        
        self.state_pub_.publish(msg)
    
    def _check_plan_update(self,):

        if self.type_ == 'static' or not self.action_mode:
            return
        
        duration: rclpy.duration.Duration = self.time - self.plan_start_time
        dt = duration.nanoseconds * 1e-9
        # self.get_logger().info("update time {}".format(dt))
        if dt > self.plan_dt:
            if len(self.plan.plan) > 0:
                self.get_logger().info("plan updated!")

                self.plan_start_time = self.time
                self.robot.velocity = self.plan.plan.pop(0)
                
                self.get_logger().info("time {}, x {} y {}, vx {}, vy {}".format(
                    dt, self.robot.pose.x, self.robot.pose.y, self.robot.velocity.vx,
                    self.robot.velocity.vy
                ))
            else:
                self.get_logger().info("uav stopped since no plan!")
                self.action_mode = False
                self.robot.velocity.vx = 0.0
                self.robot.velocity.vy = 0.0
                self.robot.velocity.omega = 0.0
    
    def _plan_callback(self, msg: Plan):
        self.get_logger().info("plan received! {}".format(msg))
        
        self.plan = msg
        self.plan_start_time = self.get_clock().now()
        self.robot.velocity = self.plan.plan.pop(0)
        self.action_mode = True

def main(args=None):
    rclpy.init(args=args)
    node = robot_node()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()