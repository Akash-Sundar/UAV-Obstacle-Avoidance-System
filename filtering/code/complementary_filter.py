# %% Imports

import numpy as np
from numpy.linalg import norm
from scipy.spatial.transform import Rotation

# %%

def __mul__(q1, q2):
    q1 = Rotation.from_quat(q1).as_matrix()
    q2 = Rotation.from_quat(q2).as_matrix()

    q = q1 @ q2
    q = Rotation.from_matrix(q).as_quat()

    return q

def adaptive_gain(e_m):

    x = [0.1, 0.2]
    y = [1, 0]

    alpha = np.interp(e_m, x, y) #also extrapolates for outer range values

    return alpha

# %%

def complementary_filter_update(initial_rotation, angular_velocity, linear_acceleration, dt):
    """
    Implements a complementary filter update

    :param initial_rotation: rotation_estimate at start of update
    :param angular_velocity: angular velocity vector at start of interval in radians per second
    :param linear_acceleration: linear acceleration vector at end of interval in meters per second squared
    :param dt: duration of interval in seconds
    :return: final_rotation - rotation estimate after update
    """

    # TODO Your code here - replace the return value with one you compute
 
    e_x = np.array([1, 0, 0])

    q_0 = initial_rotation

    q_gyro = Rotation.from_rotvec(angular_velocity * dt)
    R_k = q_0 * q_gyro

    a_k = linear_acceleration

    g_prime = R_k.as_matrix() @ a_k
    g_prime = g_prime / norm(g_prime)

    theta = np.arccos(g_prime @ e_x)

    w_acc = np.cross(g_prime, e_x)
    w_acc = w_acc / norm(w_acc)
    
    q_acc = Rotation.from_rotvec(w_acc * theta).as_quat()

    e_m = np.abs(norm(a_k)/9.81 - 1)
    q_I = np.array([ 0, 0, 0, 1])
    
    alpha = adaptive_gain(e_m)
        
    q_acc_prime = (1-alpha) * q_I + alpha * q_acc
    q_k = __mul__(q_acc_prime, R_k.as_quat())

    final_rotation = Rotation.from_quat(q_k)

    return final_rotation

# %%
