import numpy as np


def find_neighbor(current_idx, occ_map):
    graph= occ_map.map
    
    neighbor_kernel = np.array([[a,b,c] for a in range(-1,2) for b in range(-1,2) for c in range(-1,2) if (a,b,c) != (0,0,0)])
    neighbor= neighbor_kernel + current_idx

    neighbor = neighbor[np.all(neighbor >= 0, axis=1)]    
    neighbor = neighbor[np.all(neighbor < graph.shape, axis=1)]
    neighbor = neighbor[np.where(graph[neighbor[:,0], neighbor[:,1], neighbor[:,2]] == 0)]

    return neighbor



def generate_sparse_waypoints(path):

        epsilon = 0.5

        if len(path) < 3:
            return path

        dmax = 0
        idx= 0
        
        for i in range(1, len(path) - 1):
            deflection = np.linalg.norm(np.cross((path[0] - path[i]), (path[-1] - path[0]))) / np.linalg.norm(path[-1] - path[0])
            if deflection > dmax:
                idx= i
                dmax = deflection

        if dmax > epsilon:
            left = generate_sparse_waypoints(path[:idx+1])
            right = generate_sparse_waypoints(path[idx:])

            return np.vstack((left[:-1], right))

        else:
            # To be uncommented if trajectory involves traversing a long and narrow corridor. Can alternately sample points at constant intervals depending on length.
            # if np.linalg.norm(path[-1] - path[0]) > 4:
            #     return np.vstack((path[0], path[1*len(path)//4], path[2*len(path)//4], path[3*len(path)//4], path[-1]))
            
            if np.linalg.norm(path[-1] - path[0]) > 2:
                return np.vstack((path[0], path[len(path)//2], path[-1]))
            
            else:
                return np.vstack((path[0], path[-1]))

        

class min_snap(object):

    def __init__(self):
        pass

    def generate_derivative_matrix(self, t):
        arr = np.array([[0, 0, 0, 0, 0, 0, 0, 1],
                        [t**7, t**6, t**5, t**4, t**3, t**2, t, 1],
                        [7 * t**6, 6 * t**5, 5 * t**4, 4 * t**3, 3 * t**2, 2 * t, 1, 0],
                        [42 * t**5, 30 * t**4, 20 * t**3, 12 * t**2, 6 * t, 2, 0, 0],
                        [210 * t**4, 120 * t**3, 60 * t**2, 24 * t, 6, 0, 0, 0],
                        [840 * t**3, 360 * t**2, 120 * t, 24, 0, 0, 0, 0],
                        [2520 * t**2, 720 * t, 120, 0, 0, 0, 0, 0],
                        [5040 * t, 720, 0, 0, 0, 0, 0, 0]], dtype = object)
        return arr


    def generate_x_matrix(self, t, coeff_segment):
        t_arr = np.array([[t**7, t**6, t**5, t**4, t**3, t**2, t, 1],
                        [7 * t**6, 6 * t**5, 5 * t**4, 4 * t**3, 3 * t**2, 2 * t, 1, 0],
                        [42 * t**5, 30 * t**4, 20 * t**3, 12 * t**2, 6 * t, 2, 0, 0],
                        [210 * t**4, 120 * t**3, 60 * t**2, 24 * t, 6, 0, 0, 0],
                        [840 * t**3, 360 * t**2, 120 * t, 24, 0, 0, 0, 0]])

        x_matrix = t_arr @ coeff_segment

        return x_matrix
    


class min_jerk(object):

    def __init__(self):
        pass

    def generate_derivative_matrix(self, t):
        arr = np.array([[0, 0, 0, 0, 0, 1],
                        [t**5, t**4, t**3, t**2, t**1, 1],
                        [5 * t**4, 4 * t**3, 3 * t**2, 2 * t**1, 1, 0],
                        [20 * t**3, 12 * t**2, 6 * t**1, 2, 0, 0],
                        [60 * t**2, 24 * t**1, 6 , 0, 0, 0],
                        [120 * t**1, 24 , 0, 0, 0, 0]], dtype = object)
        return arr


    def generate_x_matrix(self, t, coeff_segment):
        t_arr = np.array([[t**5, t**4, t**3, t**2, t**1, 1],
                        [5 * t**4, 4 * t**3, 3 * t**2, 2 * t**1, 1, 0],
                        [20 * t**3, 12 * t**2, 6 * t**1, 2, 0, 0],
                        [60 * t**2, 24 * t**1, 6 , 0, 0, 0],
                        [120 * t**1, 24 , 0, 0, 0, 0]])

        x_matrix = t_arr @ coeff_segment

        return x_matrix


