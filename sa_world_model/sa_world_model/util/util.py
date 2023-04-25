import numpy as np
from sa_msgs.msg import Pose, Velocity

def euclidean_dist(x1: float, y1: float, x2: float, y2: float) -> float:
    return np.sqrt( (x1 - x2)**2 + (y1 - y2)**2)

def apply_action(
    pose: Pose, 
    action: Velocity, 
    dt: float
):
    """
    change the pose by action and time
    """
    pose.x += action.vx * dt
    pose.y += action.vy * dt
    pose.yaw += action.omega * dt

    return pose
