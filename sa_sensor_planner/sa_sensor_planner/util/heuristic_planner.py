import numpy as np
from typing import List


def generate_waypoints(
    list_of_points: List[List[float]],
    width: float,
    height: float,
) -> List[List[float]]:
    adj_matrix_ = adj_matrix(list_of_points, width, height)
    # print(adj_matrix_)

    clusters = []
    point_set = set(range(len(list_of_points)))

    while len(point_set) > 0:
        index = point_set.pop()
        new_cluster = set()
        new_cluster.add(index)
        traverse(index, adj_matrix_, point_set, new_cluster)
        clusters.append(new_cluster)
        point_set -= new_cluster
    # print(clusters)

    # goal_point_list is the destinations at one time stage
    goal_point_list = []

    for cluster in clusters:
        # print("start clustering the cluster of {}".format(cluster))
        candidate_cliques = triangle_cluster_north_west(cluster, list_of_points, width, height)
        for sub_cliques in candidate_cliques:
            for one_clique in sub_cliques:
                center = clique_center(one_clique, list_of_points)
                goal_point_list.append(center)

    return goal_point_list

def clique_center(indices: int, list_of_points: List[List[float]]) -> List[float]:
    center = [0.0, 0.0]
    for index in indices:
        center[0] += list_of_points[index][0] / len(indices)
        center[1] += list_of_points[index][1] / len(indices)
    
    return center


def manhattan_dist_rectangle(
    p1: List[float], 
    p2: List[float], 
    width: float, 
    height: float
) -> bool:
    return abs(p1[0]-p2[0]) < width and abs(p1[1]-p2[1]) < height

def adj_matrix(
    positions: List[List[float]], 
    width: float, 
    height: float
) -> np.ndarray:
    """
    return the adjancy matrix A + I of the positions,
    the diagoal elements are all 1 for the convenicen of later calculation.
    Adding threshold communication link distance. 
    If communication link distance > threshold_comm_distance:
        Disconnect the edge i.e replace the Distance value
        with a '0'
    
    """
    # print(positions)
    m = len(positions)
    matrix = np.zeros((m, m))
    for i in range(m):
        for j in range(m):
            #matrix[i, j] = util.euclidean(positions[i], positions[j])
            # Dist = euclidean(positions[i],positions[j])
            # if (Dist >= d_0):
            #     matrix[i,j] = 0
            # else:
            #     matrix[i,j] = 1
            
            if manhattan_dist_rectangle(positions[i], positions[j], width, height):
                matrix[i,j] = 1
            else:
                matrix[i,j] = 0

    return matrix

def traverse(index: int, adj_matrix_: np.ndarray, active_set: set, cluster: set):

    for new_index in active_set.copy():
        if adj_matrix_[index, new_index] > 0 and index != new_index:
            cluster.add(new_index)
            try: active_set.remove(new_index)
            except: pass
            traverse(new_index, adj_matrix_, active_set, cluster)

def triangle_cluster_north_west(
    cluster: set, 
    list_of_points: List[List[float]],
    width: float,
    height: float
) -> List[List[int]]:

    candidate_cliques: List[List[int]] = list()
    # edge case, only 1 element inside
    if len(cluster) < 2:
        candidate_cliques = [[list(cluster)]]
        return candidate_cliques

    cluster = list(cluster)

    
    # the indices of points sorted in decreasing X rank <
    index_sort_in_X = sorted(cluster, key=lambda index: list_of_points[index][0], reverse=True)


    # print("looking at points from left to right: {}".format(index_sort_in_X))

    while(index_sort_in_X):
        
        # the indices of points sorted in decreasing Y rank V
        index_sort_in_Y = sorted(index_sort_in_X, key=lambda index: list_of_points[index][1], reverse=True)
        sub_cliques: List[List[int]] = list()
        most_left_index = index_sort_in_X.pop()

        # print("current key {}".format(most_left_index))
        # get the indices of indices in decreasing Y points so that it fits the range
        y_max_index, y_min_index = -1, -1
        for i, index_y in enumerate(index_sort_in_Y):
            
            if y_max_index < 0:
                if list_of_points[index_y][1] - list_of_points[most_left_index][1] < height:
                    y_max_index = i
                    y_min_index = i
            else:
                y_min_index = i
                if list_of_points[most_left_index][1] - list_of_points[index_y][1] > height:
                    break
        # all points north or south of the left_most point
        # print(index_sort_in_Y, y_min_index, y_max_index)
        related_y_sorted = index_sort_in_Y[y_max_index : y_min_index+1]
        related_x_sorted = sorted(related_y_sorted, key=lambda index: list_of_points[index][0], reverse=False)
        # print(related_x_sorted, related_y_sorted)

        for index_x in related_x_sorted[::-1]:
            if list_of_points[index_x][0] - list_of_points[most_left_index][0] > width:
                related_x_sorted.pop()
            else:
                break
        
        if len(related_x_sorted) == len(index_sort_in_X) + 1:  index_sort_in_X = []
        # all points west of left_most_point, then re-organize it 
        related_y_sorted = sorted(related_x_sorted, key=lambda index: list_of_points[index][1], reverse=True)
        

        previous_one: List[int] = list()
        while related_y_sorted:
            # print("previous_one: {}, current left most{}".format(previous_one, most_left_index))
            top_y_index = related_y_sorted.pop()
            new_one: List[int] = list()
            new_one.append(top_y_index)
            for index_y in related_y_sorted:
                if list_of_points[top_y_index][1] - list_of_points[index_y][1] > height:
                    # jump outside the loop
                    break
                new_one.append(index_y)
            # analysis if this is just part of the previous_one
            if not (all(x in previous_one for x in new_one)):
                # include since it's different
                # print(new_one)
                sub_cliques.append(new_one)
                previous_one = new_one
        
        candidate_cliques.append(sub_cliques)

    # print(candidate_cliques)
    return candidate_cliques
