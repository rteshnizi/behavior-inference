from sa_msgs.msg import Pose, DetectionArray, Detection
from sa_world_model.util.sensor import sensor
from sa_world_model.util.util import euclidean_dist
from sa_world_model.util.geometry import Point, doIntersect
from typing import List, Any

class beam(sensor):

    def __init__(self, 
        x_list: List[float], 
        y_list: List[float],
        mobile: bool,
        robot_id: int,
    ):
        super().__init__(x_list, y_list, mobile, robot_id)
        self.point_cache: List[Point] = list()
        assert len(x_list) == 2, RuntimeError("beam sensor fov should be a segment")
        self.fov: List[Point] = list()
        self.fov_actual: List[Point] = list()   # what is actually used to filter obs

        for x, y in zip(x_list, y_list):
            self.fov.append(Point(x, y))
            self.fov_actual.append(Point(x, y))
    
    def _cache_point(self, msg: DetectionArray):

        self.point_cache.clear()
        for detection_ in msg.detections:
            pt = Point(detection_.pose.x, detection_.pose.y)
            self.point_cache.append(pt)
        return
    
    def _adjust_fov(self, robot_pose: Pose):
        self.robot_pose = robot_pose
        for pt_actual, pt in zip(self.fov_actual, self.fov):
            pt_actual.x = pt.x + robot_pose.x
            pt_actual.y = pt.y + robot_pose.y

    
    def _beam_sensor_fov_filter(self, msg: DetectionArray) -> bool:

        if len(self.point_cache) == 0: return False

        # here we use local nearest neighbor to analysis if the observation
        # from previous time-step to next time step cross the beam laser

        for detection_ in msg.detections:
            cur_pt = Point(detection_.pose.x, detection_.pose.y)
            dist_list = []
            for pre_pt in self.point_cache:
                dist_list.append(euclidean_dist(cur_pt.x, cur_pt.y, pre_pt.x, pre_pt.y))
            
            # find the min point
            match_pt = self.point_cache[dist_list.index(min(dist_list))]

            # check for intersection of segment
            if doIntersect(cur_pt, match_pt, self.fov_actual[0], self.fov_actual[1]):
                return True

        return False
    
    def sensor_observation(self, msg: DetectionArray, robot_pose: Pose) -> DetectionArray:
        '''
        return 
        '''
        new_msg = DetectionArray()

        if self.mobile:  self._adjust_fov(robot_pose)
        
        new_msg.beam_detection = self._beam_sensor_fov_filter(msg)
        
        self._cache_point(msg)

        return new_msg