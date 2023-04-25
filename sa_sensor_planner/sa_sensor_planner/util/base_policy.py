import numpy as np
from typing import List
import copy

def middle_point(
    list_of_points: List[List[float]],
) -> List[float]:

    new_pt = [0.0, 0.0]

    for point in list_of_points:
        
        # add the pt which is at the center of all points

        new_pt[0] += point[0] / len([list_of_points])
        new_pt[1] += point[1] / len([list_of_points])
    
    return new_pt

