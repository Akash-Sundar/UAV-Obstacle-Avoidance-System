from heapq import heappush, heappop  # Recommended.
import numpy as np

from flightsim.world import World

from collections import defaultdict

from .occupancy_map import OccupancyMap # Recommended.

from .helper import find_neighbor

def graph_search(world, resolution, margin, start, goal, astar):
    """
    Parameters:
        world,      World object representing the environment obstacles
        resolution, xyz resolution in meters for an occupancy map, shape=(3,)
        margin,     minimum allowed distance in meters from path to obstacles.
        start,      xyz position in meters, shape=(3,)
        goal,       xyz position in meters, shape=(3,)
        astar,      if True use A*, else use Dijkstra
    Output:
        return a tuple (path, nodes_expanded)
        path,       xyz position coordinates along the path in meters with
                    shape=(N,3). These are typically the centers of visited
                    voxels of an occupancy map. The first point must be the
                    start and the last point must be the goal. If no path
                    exists, return None.
        nodes_expanded, the number of nodes that have been expanded
    """

    # While not required, we have provided an occupancy map you may use or modify.
    occ_map = OccupancyMap(world, resolution, margin)
    # Retrieve the index in the occupancy grid matrix corresponding to a position in space.
    start_index = tuple(occ_map.metric_to_index(start))
    goal_index = tuple(occ_map.metric_to_index(goal))

    explored_set = set()
    parent_dict = {}


    distance_dict = defaultdict(lambda: float("inf"))
    heuristic_distance_dict = defaultdict(lambda: float("inf"))

    distance_dict[start_index] = 0
    heuristic_distance_dict[start_index] = np.linalg.norm(np.subtract(goal_index,start_index))
    nodes_in_heap = [(0, start_index)]

    while nodes_in_heap:

        _, current_idx = heappop(nodes_in_heap)

        if current_idx == goal_index:
            path = []
            node = goal_index
            path.append(goal)

            while node:
                parent = parent_dict[node]
                if parent == start_index:
                    path.insert(0, start)
                    break
                path_element = occ_map.index_to_metric_center(parent)
                path.insert(0, path_element)
                node = parent

            n = len(explored_set)
            path = np.array(path)
            
            return path, n

        explored_set.add(current_idx)
        neighbor = find_neighbor(np.array(current_idx), occ_map)

        for i in neighbor:
            i_ = tuple(i)
            g_distance = np.linalg.norm(i - current_idx)
            h_distance = np.linalg.norm(i - goal_index)
            heuristic_dist = heuristic_distance_dict[current_idx]
            heuristic_dist = heuristic_dist + g_distance

            dist = heuristic_dist + h_distance * astar

            if dist < distance_dict[i_]:
                distance_dict[i_] = dist
                heuristic_distance_dict[i_] = heuristic_dist
                parent_dict[i_] = current_idx
                heappush(nodes_in_heap, (dist, i_))
  
    # Return a tuple (path, nodes_expanded)
    return None, 0
