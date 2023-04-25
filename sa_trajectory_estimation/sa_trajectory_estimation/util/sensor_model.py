import numpy as np
from typing import List

def range_bearing_cov(z: List[float], 
    xs: List[float],
    alpha: float,
    r0: float,
) -> np.ndarray:
    # xs: sensor pos, = [x, y, theta]
    # z: measurement, [x, y] 
    dx = z[0] - xs[0]
    dy = z[1] - xs[1]
    theta = np.arctan2(dy, dx) - xs[2]
    r = max(r0, np.sqrt(dx**2 + dy**2))
    G = np.matrix([[np.cos(theta), -np.sin(theta)], 
                    [np.sin(theta), np.cos(theta)]])
    M = np.diag([0.1 * r, 0.1 * np.pi * r])
    R = alpha * np.dot(np.dot(G, M), G.T)
    
    return R