#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from typing import List, Optional
import json, os
from sa_msgs.msg import RobotState, PoseArray, EstimationMsg, Plan, VisibilityArray
from sa_msgs.srv import QueryVisibility
from ament_index_python.packages        import get_package_share_directory
from sa_sensor_planner.util.single_uav_planner import single_sensor_nbo
from shapely import geometry
from threading import Event

class sensor_planner_node(Node):
    def __init__(self):
        super().__init__('sensor_planner_node')
        
        
        self.declare_parameter('plan_frequency', 1.0)
        self.declare_parameter('dynamic_object_list', ['car', 'truck'])
        self.declare_parameter('static_object_list', [])
        self.declare_parameter('interest_object_list', ['car', 'truck'])
        self.declare_parameter('robot_name_list', ['uav1', 'beam_static'])
        self.declare_parameter('horizon', 5)
        self.declare_parameter('robot_id', 1)
        self.declare_parameter('v_max', 5.0)
        # Attribute ===============================================
        
        plan_frequency = self.get_parameter('plan_frequency').get_parameter_value().double_value
        self.dynamic_object_list = self.get_parameter('dynamic_object_list').get_parameter_value().string_array_value
        self.static_object_list = self.get_parameter('static_object_list').get_parameter_value().string_array_value
        self.interest_object_list = self.get_parameter('interest_object_list').get_parameter_value().string_array_value
        self.robot_name_list = self.get_parameter('robot_name_list').get_parameter_value().string_array_value
        self.horizon = self.get_parameter('horizon').get_parameter_value().integer_value
        self.plan_dt = 1 / plan_frequency
        self.robot_id = self.get_parameter('robot_id').get_parameter_value().integer_value
        v_max = self.get_parameter('v_max').get_parameter_value().double_value
        self._load_sensor_para()
        # self.get_logger().info("{}".format(self.dynamic_object_list))
        # self.get_logger().info("{}".format(self.robot_name_list))

        self.uav_planner = single_sensor_nbo(
            self.sensor_para_list["sensors"],
            self.dynamic_object_list,
            self.static_object_list,
            self.plan_dt,
            self.horizon,
            self.interest_object_list,
            self.robot_id,
            v_max
        )
    
        # publishers =============
        # self.get_logger().info("sensor planner {}".format(self.get_namespace() + '/plan'))
        self.create_timer(plan_frequency, self.motion_plan_callback)
        self.sensor_plan_pub_ = self.create_publisher(
            Plan, 
            self.get_namespace() + '/plan', 
            10)

        # subscribers ==============

        
        self.estimation = EstimationMsg()
        
        for robot_name in self.robot_name_list:
            self.create_subscription(
                RobotState,
                '/' + robot_name + '/state',
                self.robot_state_callback,
                10,
            )

        self.create_subscription(
            EstimationMsg,
            '/sa_tajectory_estimation/estimation',
            self.trajectory_estimation_callback,
            10,
        )

        # client
        self._dummy_occlusion()
        self.action_done_event = Event()
        self.response = None

    def _dummy_occlusion(self):
        point_list = [[-5.83, 47.7189], 
                      [2.8314, 13.0715], 
                      [51.80668, 25.3153],
                      [43.1448, 59.9627]]
        self.occlusion = geometry.Polygon(point_list)
    
    def _get_duration(self, time1, time2):
        duration: rclpy.duration.Duration = time2 - time1
        dt = duration.nanoseconds * 1e-9
        return dt
    
    def motion_plan_callback(self):
        time0 = self.get_clock().now()
        # 1. propagate the NBO traj estimation
        traj_array = self.uav_planner.sample(self.estimation.pose_estimations)
        for est in self.estimation.pose_estimations:
            self.get_logger().info("{}, {}, {}, {}, {}".format(
                est.pose.x, est.pose.y, est.velocity.vx, est.velocity.vy,
                est.detection_label
            ))

        # potential issue of using ros2 server
        # 2. obtain the visibility 
        self.get_logger().info("requested visibility")
        time1 = self.get_clock().now()
        
        # self._request_visibility(traj_array)
        # visibility = self.response
        # self.get_logger().info("{}".format(visibility))
        # visibility = response.visibilities
            
        visibility = self._dummy_visibility(traj_array)
        self.get_logger().info("srv finished")
        time2 = self.get_clock().now()

        # 3. generate the plan
        plan = self.uav_planner.plan(visibility)

        self.sensor_plan_pub_.publish(plan)

        time3 = self.get_clock().now()
        self.get_logger().info("plan finished, srv time {}, \
                planning whole duration {}".format(self._get_duration(time1, time2),
                                                   self._get_duration(time0, time3)))
    
    def _dummy_visibility(self, traj_array: List[PoseArray]) -> List[VisibilityArray]:
        
        visibility: List[VisibilityArray] = list()

        # for traj in traj_array:

        for t in range(self.horizon):
            traj_visi = VisibilityArray()
            for traj in traj_array[t].traj:
                traj_visi.visibilities.append(self._is_visibile(traj.x, traj.y))
            visibility.append(traj_visi)
        return visibility

    
    def _request_visibility(self, traj_array: List[PoseArray]) -> Optional[List[VisibilityArray]]:
        # https://github.com/ros2/rclpy/issues/524
        # TODO check if this works

        visibility_client = self.create_client(QueryVisibility, 
                            self.get_namespace() + '/visibility_query')
        

        # response = visibility_client.call(request) 
        # simply using sync call will cause deadlock
        # https://answers.ros.org/question/343279/ros2-how-to-implement-a-sync-service-client-in-a-node/


        # https://github.com/ros2/examples/blob/foxy/rclpy/services/minimal_client/examples_rclpy_minimal_client/client_async_callback.py
        # this tutorial never gets service running.


        async def call_service(visibility_client, request):
            visibility_future = visibility_client.call_async(request)
            self.get_logger().info('sent service')
            # visibility_future.add_done_callback(self._visibility_clinet_callback)

            try:
                result = await visibility_future
            except Exception as e:
                self.get_logger().info('Service call failed %r' % (e,))
            else:
                self.get_logger().info(
                        'received reuslt')
                self.response = result.visibility_array

        while not visibility_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().info('service not available, waiting again...')
        
        request = QueryVisibility.Request()

        request.traj_array = traj_array
        call_service(visibility_client, request)

        # https://answers.ros.org/question/322831/ros2-wait-for-action-using-rclpy/
        # this method does not work either
    

    def robot_state_callback(self, msg: RobotState):

        self.uav_planner.update_robot_state(msg)

    def _is_visibile(self, x: float, y: float) -> bool:

        # check if one detection is inside occlusion
        a_point = geometry.Point(x, y)
        if a_point.within(self.occlusion):
            
            return False
        else:   return True
    
    def _load_sensor_para(self):
        path_ = os.path.join(get_package_share_directory('sa_trajectory_estimation'),'config','sensor.json')    
        with open(path_) as json_file:
                
            self.sensor_para_list = json.load(json_file)


    def trajectory_estimation_callback(self, msg: EstimationMsg):
        '''estimation recieved validated'''
        # self.get_logger().info("node received with time %s"%msg)
        self.estimation = msg
    
    
    


def main(args=None):
    rclpy.init(args=args)
    node = sensor_planner_node()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()