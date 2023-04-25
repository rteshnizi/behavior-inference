from sa_msgs.msg import Pose, Velocity, Plan
from sa_world_model.util.robot import robot
from sa_world_model.util.util import apply_action

class uav(robot):

    def __init__(self, x: float, y: float, yaw: float) -> None:
        super().__init__(x, y, yaw)

    def _dynamics(self, dt: float):

        self.pose = apply_action(self.pose, self.velocity, dt)