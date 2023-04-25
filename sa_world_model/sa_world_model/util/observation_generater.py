import numpy as np
from typing import List

def linear_traj(t: float) -> List[float]:
    return [1.0, 1.0]

def demo_traj(t: float) -> List[List[float]]:

    car_pose = car_pos(t, 5)

    tank_pose = tank_pos(t, 4.8)

    return [car_pose, tank_pose]


def car_pos(
    t: float,
    v: float,
) -> List[float]:
    
    slope = 0.2568
    t0 = 147.7 * np.sqrt(1 + slope**2) / v
    
    if t < t0:

        # l1
        slope = 0.2568
        dx = v * t / np.sqrt(1 + slope**2)

        x = -125.5 + dx
        y = -3.88 + dx * slope
    
    else:
        dx = v * (t - t0) / np.sqrt(17)
        x = 22 + dx
        y = 4 * dx + 34.0
    return [x, y, 'car']

def tank_pos(
    t: float,
    v: float,
) -> List[float]:
    
    slope = 0.2501
    
    dx = v * t / np.sqrt(1 + slope**2)

    x = -125.5 + dx
    y = -8.88 + dx * slope
    return [x, y, 'tank']