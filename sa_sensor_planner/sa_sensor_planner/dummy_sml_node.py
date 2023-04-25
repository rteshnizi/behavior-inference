from sa_msgs.msg import RobotState, PoseArray, EstimationMsg, Plan, VisibilityArray
from sa_msgs.srv import QueryVisibility
from typing import List
import rclpy
from rclpy.node import Node
from shapely import geometry

class MinimalService(Node):

    def __init__(self):
        super().__init__('dummy_sml_service')
        self._dummy_occlusion()
        
        self.srv = self.create_service(QueryVisibility, 
                        self.get_namespace() + '/visibility_query', self.visibility_query_callback)        # CHANGE

    def _dummy_occlusion(self):
        point_list = [[-5.83, 47.7189], 
                      [2.8314, 13.0715], 
                      [51.80668, 25.3153],
                      [43.1448, 59.9627]]
        self.occlusion = geometry.Polygon(point_list)

    def visibility_query_callback(self, request, response):
        # response.sum = request.a + request.b + request.c   
        self.get_logger().info('request comming') # CHANGE
        
        for traj_ in request.traj_array:
            traj_visi = VisibilityArray()

            for traj in traj_.traj:
                traj_visi.visibilities.append(self._is_visibile(traj.x, traj.y))

            response.visibility_array.append(traj_visi)
        self.get_logger().info('task finished') 
        return response

    def _is_visibile(self, x: float, y: float) -> bool:

        # check if one detection is inside occlusion
        a_point = geometry.Point(x, y)
        if a_point.within(self.occlusion):
            
            return False
        else:   return True

def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()