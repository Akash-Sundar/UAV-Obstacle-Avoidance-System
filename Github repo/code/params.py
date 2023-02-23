import numpy as np

class world_traj(object):
    def __init__(self):
        self.resolution = np.array([0.25, 0.25, 0.25])
        self.margin = 0.5
        self.v = 10

        self.first_and_last_time_segment_scaling_factor = 2
        self.time_scaling_coeff = 1.65

        self.trajectory_type = 'min_snap' #choose between 'min_snap', 'min_jerk' and 'constant_speed'. 

class se3(object):
    def __init__(self, trajectory_type = 'min_snap'):
        if trajectory_type == 'min_snap':  
            self.K_p = np.diag([8, 8, 20])
            self.K_d = np.diag([6.2, 6.2, 9])
            self.K_R = np.diag([2800,2800,160])
            self.K_w = np.diag([125,125,75])

        elif trajectory_type == 'min_jerk': #still need to fine tune gains for min_jerk
            self.K_p = np.diag([7, 7, 15])
            self.K_d = np.diag([5.5, 5.5, 7])
            self.K_R = np.diag([2400,2400,140])
            self.K_w = np.diag([120,120,60])

        elif trajectory_type == 'constant_speed':
            self.K_p = np.diag([5.2, 5.2, 7])
            self.K_d = np.diag([4.4, 4.4, 2.5])
            self.K_R = np.diag([1800,1800,40])
            self.K_w = np.diag([90,90,5])


