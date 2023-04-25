from sa_msgs.msg import Pose, DetectionArray, Detection, Velocity
from sa_world_model.util.sensor import sensor
from typing import List, Any
import shapely
from shapely import geometry
from sa_world_model.util.util import apply_action
from sa_trajectory_estimation.util.sensor_model import range_bearing_cov
import numpy as np

class camera(sensor):

    def __init__(
        self, 
        x_list: List[float], 
        y_list: List[float],
        mobile: bool,
        robot_id: int,
        quality: float,
        r0: float,
        robot_pose: float,
    ):
        super().__init__(x_list, y_list, mobile, robot_id)
        self.fov = geometry.Polygon([[x, y] for x, y in zip(x_list, y_list)])
        self._adjust_fov(robot_pose)
        self.quality = quality
        self.r0 = r0
    
    def _adjust_fov(self, robot_pose: Pose):

        # translate by robot position, no rotation currently applied
        self.robot_pose = robot_pose
        self.fov_actual = shapely.affinity.translate(
            self.fov, 
            xoff = robot_pose.x, 
            yoff = robot_pose.y)
    
    def move_sensor(
        self,
        action: Velocity, 
        dt: float,
    ):
        self.robot_pose = apply_action(self.robot_pose, action, dt)
        self._adjust_fov(self.robot_pose)


    def _is_in_fov(self, x: float, y: float) -> bool:

        # check if one detection is within fov
        a_point = geometry.Point(x, y)
        if a_point.within(self.fov_actual):
            
            return True
        else:   return False

    def _add_noise(self, detection_array: DetectionArray) -> DetectionArray:
        '''add Gaussian White noise based on sensor model'''

        for detection_ in detection_array.detections:
            z = [detection_.pose.x, detection_.pose.y]
            xs = [self.robot_pose.x, self.robot_pose.y, self.robot_pose.yaw]

            cov = range_bearing_cov(z, xs, self.quality, self.r0)
            vk = np.random.multivariate_normal(mean=np.zeros(2), cov=cov, size=1)
                    
            detection_.pose.x += vk[0, 0]
            detection_.pose.x += vk[0, 1]
        return detection_array