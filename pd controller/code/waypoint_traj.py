import numpy as np


class WaypointTraj(object):
    """

    """

    def __init__(self, points):
        """
        This is the constructor for the Trajectory object. A fresh trajectory
        object will be constructed before each mission. For a waypoint
        trajectory, the input argument is an array of 3D destination
        coordinates. You are free to choose the times of arrival and the path
        taken between the points in any way you like.

        You should initialize parameters and pre-compute values such as
        polynomial coefficients here.

        Inputs:
            points, (N, 3) array of N waypoint coordinates in 3D
        """

        # STUDENT CODE HERE
        self.points = points
        self.N = len(points)

        self.pi = points[:self.N - 1]
        self.pi_ = points[1:self.N]
        self.dir = self.pi - self.pi_

        self.d = np.linalg.norm(self.dir, axis=1).reshape(-1,1)
        self.unit_dir = -self.dir / self.d

        self.t0 = 0
        self.v = 2.5

        self.ti = self.d / self.v
        # print('self.ti: ', self.ti)
        self.T = np.cumsum(self.ti)
        self.T = np.hstack([self.t0, self.T])

    def update(self, t):
        """
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
        x = np.zeros((3,))
        x_dot = np.zeros((3,))
        x_ddot = np.zeros((3,))
        x_dddot = np.zeros((3,))
        x_ddddot = np.zeros((3,))
        yaw = 0
        yaw_dot = 0

        # STUDENT CODE HERE

        tmp = -1
        for i in range(0, self.T.shape[0] - 1):
             if self.T[i] <= t and t < self.T[i + 1]:
                tmp = i
                break
        i = tmp

        # if t == 0:
        #     x_dot = np.zeros(3,)
        #     x = np.zeros(3,)

        if i >= 0:
            x_dot = self.v * self.unit_dir[i]
            x = self.pi[i] + x_dot * (t - self.T[i])

        else:
            x_dot = np.zeros(3)
            x = self.points[-1]

        flat_output = {'x': x, 'x_dot': x_dot, 'x_ddot': x_ddot, 'x_dddot': x_dddot, 'x_ddddot': x_ddddot,
                       'yaw': yaw, 'yaw_dot': yaw_dot}
        # print(flat_output)
        return flat_output
