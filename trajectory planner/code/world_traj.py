import numpy as np
from scipy.sparse.linalg import spsolve
from scipy.sparse import lil_matrix as sparse_matrix

from .graph_search import graph_search


from .helper import generate_sparse_waypoints, min_snap, min_jerk
from .params import world_traj
params = world_traj()


class WorldTraj(object):
    """
    """

            
    def __init__(self, world, start, goal):
        """
        This is the constructor for the trajectory object. A fresh trajectory
        object will be constructed before each mission. For a world trajectory,
        the input arguments are start and end positions and a world object. You
        are free to choose the path taken in any way you like.
        You should initialize parameters and pre-compute values such as
        polynomial coefficients here.
        Parameters:
            world, World object representing the environment obstacles
            start, xyz position in meters, shape=(3,)
            goal,  xyz position in meters, shape=(3,)
        """
        # You must choose resolution and margin parameters to use for path
        # planning. In the previous project these were provided to you; now you
        # must chose them for yourself. Your may try these default values, but
        # you should experiment with them!
        self.resolution = params.resolution
        self.margin = params.margin
        # You must store the dense path returned from your Dijkstra or AStar
        # graph search algorithm as an object member. You will need it for
        # debugging, it will be used when plotting results.
        self.path, _ = graph_search(world, self.resolution, self.margin, start, goal, astar=True)

        self.points = generate_sparse_waypoints(self.path)       

        # print(len(self.path))
        print('Number of Waypoints: ', len(self.points))

        self.v = params.v

        self.N = len(self.points)
        self.pi = self.points[:self.N - 1]
        self.pi_ = self.points[1:self.N]
        self.dir = self.pi - self.pi_
        self.d = np.linalg.norm(self.dir, axis=1).reshape((-1,1))
        self.unit_dir = -self.dir/self.d

        self.time = self.d / self.v
        self.time[0] = params.first_and_last_time_segment_scaling_factor*self.time[0]
        self.time[-1] = params.first_and_last_time_segment_scaling_factor*self.time[-1]
        self.time = self.time * np.sqrt(params.time_scaling_coeff / self.time)

        self.T = np.vstack((np.zeros(1), np.cumsum(self.time, axis=0))).flatten()

    ## Minimum Snap
    def min_snap(self, t):
        trajectory = min_snap()
        x_matrix = np.zeros((5,3))
        num_segments = 8 * (self.N-1)

        A = sparse_matrix((num_segments, num_segments))
        b = sparse_matrix((num_segments, 3))

        t_final = np.sum(self.time)

        for i in range(0, self.N - 1):
            b[8 * i + 3] = self.points[i]
            b[8 * i + 4] = self.points[i + 1]

        
        for ti, i in zip(self.time, range(0, len(self.time))):

            sub_matrix = trajectory.generate_derivative_matrix(ti)

            A[0,6] = 1
            A[1,5] = 2
            A[2,4] = 6

            if i == len(self.time) - 1:
                A[8*i + 3:8*i + 11, 8*i:8*i + 8] = sub_matrix[:5, :]
            
            else:
                A[8*i + 5, 8*i + 14] = -1
                A[8*i + 6, 8*i + 13] = -2
                A[8*i + 7, 8*i + 12] = -6
                A[8*i + 8, 8*i + 11] = -24
                A[8*i + 9, 8*i + 10] = -120
                A[8*i + 10, 8*i + 9] = -720

                A[8*i + 3:8*i + 11, 8*i:8*i + 8] = sub_matrix

        A = A.tocsc()
        b = b.tocsc()
        coeff = spsolve(A, b).toarray()

        if t > t_final:
            x_matrix[0] = self.points[-1]
        else:
            segment_number = np.where(self.T > t)
            segment_number = segment_number[0][0] - 1
            t_segment_start = self.T[segment_number]
            t_ = t - t_segment_start

            x_matrix = trajectory.generate_x_matrix(t_, coeff[8*segment_number : 8*(segment_number + 1)])

        return x_matrix
    

    ## Minimum Jerk
    def min_jerk(self, t):
        trajectory = min_jerk()
        x_matrix = np.zeros((5,3))
        num_segments = 6 * (self.N-1)

        A = sparse_matrix((num_segments, num_segments))
        b = sparse_matrix((num_segments, 3))

        t_final = np.sum(self.time)

        for i in range(0, self.N - 1):
            b[6 * i + 2] = self.points[i]
            b[6 * i + 3] = self.points[i + 1]

        
        for ti, i in zip(self.time, range(0, len(self.time))):

            sub_matrix = trajectory.generate_derivative_matrix(ti)

            A[0,4] = 1
            A[1,3] = 2

            if i == len(self.time) - 1:
                A[6*i + 2:6*i + 8, 6*i:6*i + 6] = sub_matrix[:4, :]
            
            else:
                A[6*i + 3, 6*i + 11] = -1
                A[6*i + 4, 6*i + 10] = -2
                A[6*i + 5, 6*i + 9] = -6
                A[6*i + 6, 6*i + 8] = -24

                A[6*i + 2:6*i + 8, 6*i:6*i + 6] = sub_matrix

        A = A.tocsc()
        b = b.tocsc()
        coeff = spsolve(A, b).toarray()

        if t > t_final:
            x_matrix[0] = self.points[-1]
        else:
            segment_number = np.where(self.T > t)
            segment_number = segment_number[0][0] - 1
            t_segment_start = self.T[segment_number]
            t_ = t - t_segment_start

            x_matrix = trajectory.generate_x_matrix(t_, coeff[6*segment_number : 6*(segment_number + 1)])

        return x_matrix

    
    ## Constant speed    
    def constant_speed(self, t):
        x_matrix = np.zeros((5,3))
        tmp = -1
        for i in range(0, self.T.shape[0] - 1):
             if self.T[i] <= t and t < self.T[i + 1]:
                tmp = i
                break
        i = tmp

        if i >= 0:
            x_matrix[1] = self.v * self.unit_dir[i]
            x_matrix[0] = self.pi[i] + x_matrix[1] * (t - self.T[i])

        else:
            x_matrix[0] = self.points[-1]

        return x_matrix


    def update(self, t, trajectory_type = params.trajectory_type):
        """
        PRIMARY METHOD
        Given the present time, return the desired flat output and derivatives.
        Inputs
            t, time, s
        Outputs
            flat_output, a dict describing the present desired flat outputs with keys
                x,        position, m
                x_dot,    velocity, m/s
                x_ddot,   acceleration, m/s**2
                x_dddot,  jerk, m/s**3
                x_ddddot, snap, m/s**4
                yaw,      yaw angle, rad
                yaw_dot,  yaw rate, rad/s
        """
        x        = np.zeros((3,))
        x_dot    = np.zeros((3,))
        x_ddot   = np.zeros((3,))
        x_dddot  = np.zeros((3,))
        x_ddddot = np.zeros((3,))
        yaw = 0
        yaw_dot = 0

        # STUDENT CODE HERE
        if trajectory_type == 'min_snap':
            x_matrix = self.min_snap(t)

        elif trajectory_type == 'min_jerk':
            x_matrix = self.min_jerk(t)

        elif trajectory_type == 'constant_speed':
            x_matrix = self.constant_speed(t)

        else:
            x_matrix = np.zeros((5,3))
            print('Invalid Trajectory Type')
        
        x = x_matrix[0]
        x_dot = x_matrix[1]
        x_ddot = x_matrix[2]
        x_dddot = x_matrix[3]
        x_ddddot = x_matrix[4]


        # STUDENT CODE END
        flat_output = { 'x':x, 'x_dot':x_dot, 'x_ddot':x_ddot, 'x_dddot':x_dddot, 'x_ddddot':x_ddddot,
                        'yaw':yaw, 'yaw_dot':yaw_dot}
        return flat_output