from sa_msgs.msg import Pose, DetectionArray, Detection
from typing import List, Any


class sensor:

    def __init__(
        self, 
        x_list: List[float], 
        y_list: List[float],
        mobile: bool,
        robot_id: int,
    ) -> None:
        # def the sensor parameters

        self.robot_pose = Pose()
        self.fov = None
        self.mobile = mobile
        self.robot_id = robot_id
    
    def sensor_observation(self, msg: DetectionArray, robot_pose: Pose) -> DetectionArray:
        
        new_msg = DetectionArray()

        if self.mobile:  self._adjust_fov(robot_pose)
        
        for detection_ in msg.detections:
            if self._is_in_fov(detection_.pose.x, detection_.pose.y):
                new_msg.detections.append(detection_)

        return self._add_noise(new_msg)

    
    def _adjust_fov(self, robot_pose: Pose):

        # translate by robot position, no rotation currently applied
        self.robot_pose = robot_pose

    def _is_detection_in_fov(self, detection_: Detection) -> bool:

        return self._is_in_fov(detection_.pose.x, detection_.pose.y)

    def _is_in_fov(self, x: float, y: float) -> bool:

        # check if one detection is within fov
        return True
    
    def _add_noise(self, detection_array: DetectionArray) -> DetectionArray:
        '''add noise based on sensor model'''
        # you might need robot pose here
        return detection_array