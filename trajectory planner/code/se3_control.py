import numpy as np
from scipy.spatial.transform import Rotation
from .params import se3, world_traj

tmp = world_traj()
params = se3(tmp.trajectory_type)
del tmp


class SE3Control(object):
    """

    """

    def __init__(self, quad_params):
        """
        This is the constructor for the SE3Control object. You may instead
        initialize any parameters, control gain values, or private state here.

        For grading purposes the controller is always initialized with one input
        argument: the quadrotor's physical parameters. If you add any additional
        input arguments for testing purposes, you must provide good default
        values!

        Parameters:
            quad_params, dict with keys specified by crazyflie_params.py

        """

        # Quadrotor physical parameters.
        self.mass = quad_params['mass']  # kg
        self.Ixx = quad_params['Ixx']  # kg*m^2
        self.Iyy = quad_params['Iyy']  # kg*m^2
        self.Izz = quad_params['Izz']  # kg*m^2
        self.arm_length = quad_params['arm_length']  # meters
        self.rotor_speed_min = quad_params['rotor_speed_min']  # rad/s
        self.rotor_speed_max = quad_params['rotor_speed_max']  # rad/s
        self.k_thrust = quad_params['k_thrust']  # N/(rad/s)**2
        self.k_drag = quad_params['k_drag']  # Nm/(rad/s)**2

        # You may define any additional constants you like including control gains.
        self.inertia = np.diag(np.array([self.Ixx, self.Iyy, self.Izz]))  # kg*m^2
        self.g = 9.81  # m/s^2

        # STUDENT CODE HERE


    def update(self, t, state, flat_output):
        """
        This function receives the current time, true state, and desired flat
        outputs. It returns the command inputs.

        Inputs:
            t, present time in seconds
            state, a dict describing the present state with keys
                x, position, m
                v, linear velocity, m/s
                q, quaternion [i,j,k,w]
                w, angular velocity, rad/s
            flat_output, a dict describing the present desired flat outputs with keys
                x,        position, m
                x_dot,    velocity, m/s
                x_ddot,   acceleration, m/s**2
                x_dddot,  jerk, m/s**3
                x_ddddot, snap, m/s**4
                yaw,      yaw angle, rad
                yaw_dot,  yaw rate, rad/s

        Outputs:
            control_input, a dict describing the present computed control inputs with keys
                cmd_motor_speeds, rad/s
                cmd_thrust, N (for debugging and laboratory; not used by simulator)
                cmd_moment, N*m (for debugging; not used by simulator)
                cmd_q, quaternion [i,j,k,w] (for laboratory; not used by simulator)
        """
        cmd_motor_speeds = np.zeros((4,))
        cmd_thrust = 0
        cmd_moment = np.zeros((3,))
        cmd_q = np.zeros((4,))

        # STUDENT CODE HERE
        K_p = params.K_p
        K_d = params.K_d
        K_R = params.K_R
        K_w = params.K_w

        R = (Rotation.from_quat(state['q'])).as_matrix()

        r_ddot_des = (flat_output['x_ddot'].reshape(3,1)) \
                     - (K_d @ (state['v'].reshape(3,1) - flat_output['x_dot'].reshape(3,1))) \
                     - (K_p @ (state['x'].reshape(3,1) - flat_output['x'].reshape(3,1)))

        F_des = self.mass * r_ddot_des + np.array([0, 0, self.mass * self.g]).reshape(3,1)
        b = R @ np.array([0, 0, 1]).T
        u1 = b.T @ F_des

        b3_des = F_des / np.linalg.norm(F_des)
        psi_t = flat_output['yaw']
        a_psi = np.array([np.cos(psi_t), np.sin(psi_t), 0])
        b2_des = np.cross(b3_des, a_psi, axis = 0)
        b2_des = b2_des / np.linalg.norm(b2_des)

        R_des = np.hstack([np.cross(b2_des, b3_des, axis = 0), b2_des, b3_des])

        tmp = 0.5*(R_des.T @ R - R.T @ R_des)
        e_R = np.array([-tmp[1,2], tmp[0,2], -tmp[0,1]]).T
        w_des = np.zeros_like(state['w'])
        e_w = state['w'] - w_des

        u2 = self.inertia @ (-K_R @ e_R - K_w @ e_w)

        l = self.arm_length

        k_t = self.k_thrust
        k_d = self.k_drag
        gamma = k_d/k_t

        #A = np.array([[k_t, k_t, k_t, k_t], [0, l*k_t, 0, -l*k_t], [-l*k_t, 0, l*k_t, 0], [k_d, -k_d, k_d, -k_d]]) / k_t
        A = np.array([[1, 1, 1, 1], [0, l, 0, -l], [-l, 0, l, 0], [gamma, -gamma, gamma, -gamma]])
        u = np.vstack((u1.reshape(1,1), u2.reshape((-1,1))))

        F = np.linalg.inv(A) @ u

        w = np.sign(F) * np.sqrt(np.absolute(F) / k_t)

        w = np.clip(w, self.rotor_speed_min, self.rotor_speed_max)


        cmd_motor_speeds = w
        cmd_thrust = u1
        cmd_moment = u2
        cmd_q = state['q']

        # print('u1: ', u1)
        # print('u2: ', u2)

        control_input = {'cmd_motor_speeds': cmd_motor_speeds,
                         'cmd_thrust': cmd_thrust,
                         'cmd_moment': cmd_moment,
                         'cmd_q': cmd_q}

        #print(control_input)

        return control_input
