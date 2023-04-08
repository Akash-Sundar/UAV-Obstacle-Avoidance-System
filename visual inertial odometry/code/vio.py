#%% Imports

import numpy as np
from numpy.linalg import inv
from numpy.linalg import norm
from scipy.spatial.transform import Rotation


#%% Functions

def nominal_state_update(nominal_state, w_m, a_m, dt):
    """
    function to perform the nominal state update

    :param nominal_state: State tuple (p, v, q, a_b, w_b, g)
                    all elements are 3x1 vectors except for q which is a Rotation object
    :param w_m: 3x1 vector - measured angular velocity in radians per second
    :param a_m: 3x1 vector - measured linear acceleration in meters per second squared
    :param dt: duration of time interval since last update in seconds
    :return: new tuple containing the updated state
    """
    # Unpack nominal_state tuple
    p, v, q, a_b, w_b, g = nominal_state

    # YOUR CODE HERE
    new_p = np.zeros((3, 1))
    new_v = np.zeros((3, 1))
    new_q = Rotation.identity()

    R = Rotation.as_matrix(q)

    a_corrected = a_m - a_b
    w_corrected = (w_m - w_b).reshape(-1,)

    a_final = R @ a_corrected + g
    
    delta_p = v*dt + 0.5 * a_final * dt**2
    delta_v = a_final * dt
    delta_q = Rotation.from_rotvec(w_corrected*dt)

    new_p = p + delta_p
    new_v = v + delta_v
    new_q = q * delta_q

    return new_p, new_v, new_q, a_b, w_b, g


def error_covariance_update(nominal_state, error_state_covariance, w_m, a_m, dt,
                            accelerometer_noise_density, gyroscope_noise_density,
                            accelerometer_random_walk, gyroscope_random_walk):
    """
    Function to update the error state covariance matrix

    :param nominal_state: State tuple (p, v, q, a_b, w_b, g)
                        all elements are 3x1 vectors except for q which is a Rotation object
    :param error_state_covariance: 18x18 initial error state covariance matrix
    :param w_m: 3x1 vector - measured angular velocity in radians per second
    :param a_m: 3x1 vector - measured linear acceleration in meters per second squared
    :param dt: duration of time interval since last update in seconds
    :param accelerometer_noise_density: standard deviation of accelerometer noise
    :param gyroscope_noise_density: standard deviation of gyro noise
    :param accelerometer_random_walk: accelerometer random walk rate
    :param gyroscope_random_walk: gyro random walk rate
    :return:
    """

    # Unpack nominal_state tuple
    p, v, q, a_b, w_b, g = nominal_state

    sigma_a_n = accelerometer_noise_density
    sigma_w_n = gyroscope_noise_density

    sigma_a_w = accelerometer_random_walk
    sigma_w_w = gyroscope_random_walk

    P = error_state_covariance

    # YOUR CODE HERE   

    I = np.identity(3)
    dt_I = dt*I
    R = Rotation.as_matrix(q)
    a_corrected = (a_m - a_b).reshape((-1,))
    w_corrected = (w_m - w_b).reshape(-1,)

    def construct_Fx():

        a_corrected_skew = np.array([0, -a_corrected[2], a_corrected[1], 
                                    a_corrected[2], 0, -a_corrected[0], 
                                    -a_corrected[1], a_corrected[0], 0], dtype = float).reshape(3,3)

        R_w = Rotation.from_rotvec(w_corrected * dt).as_matrix()

        Fx = np.identity(18)
        Fx[0:3, 3:6] = Fx[3:6, 15:18] = dt_I
        Fx[6:9, 12:15] = -dt_I
        Fx[3:6, 6:9] = -R @ a_corrected_skew * dt
        Fx[3:6, 9:12] = -R * dt
        Fx[6:9, 6:9] = R_w.T

        return Fx

    def construct_Fi():
        zero = np.zeros((3, 12))
        Fi = np.identity(12)
        Fi = np.vstack((zero, Fi, zero))

        return Fi

    def construct_Qi():
        Qi = np.zeros((12, 12))
        Qi[0:3, 0:3] = sigma_a_n**2 * dt**2 * I
        Qi[3:6, 3:6] = sigma_w_n**2 * dt**2 * I
        Qi[6:9, 6:9] = sigma_a_w**2 * dt * I
        Qi[9:12, 9:12] = sigma_w_w**2 * dt * I

        return Qi

    Fx = construct_Fx()
    Fi = construct_Fi()
    Qi = construct_Qi()
        
    P_updated = Fx @ P @ Fx.T + Fi @ Qi @ Fi.T

    # return an 18x18 covariance matrix
    assert P_updated.shape == (18,18)
    return P_updated


def measurement_update_step(nominal_state, error_state_covariance, uv, Pw, error_threshold, Q):
    """
    Function to update the nominal state and the error state covariance matrix based on a single
    observed image measurement uv, which is a projection of Pw.

    :param nominal_state: State tuple (p, v, q, a_b, w_b, g)
                        all elements are 3x1 vectors except for q which is a Rotation object
    :param error_state_covariance: 18x18 initial error state covariance matrix
    :param uv: 2x1 vector of image measurements
    :param Pw: 3x1 vector world coordinate
    :param error_threshold: inlier threshold
    :param Q: 2x2 image covariance matrix
    :return: new_state_tuple, new error state covariance matrix
    """
    
    # Unpack nominal_state tuple
    p, v, q, a_b, w_b, g = nominal_state
    Sigma_old = error_state_covariance
    Q_t = Q

    # YOUR CODE HERE - compute the innovation next state, next error_state covariance

    I = np.identity(18)
    R = Rotation.as_matrix(q)

    Pc = R.T @ (Pw - p)
    Pc = Pc.reshape((-1,))

    Zc = Pc[-1]
    Pc_normalized = Pc / Zc

    u1, v1 = Pc_normalized[0:2]
    innovation = uv - np.array([u1, v1]).reshape((2, 1))

    if np.linalg.norm(innovation) > error_threshold:
        return (p, v, q, a_b, w_b, g), error_state_covariance, innovation
    
    else:

        def get_derivatives():

            deriv_Pc_wrt_theta = np.array([0, -Pc[2], Pc[1], 
                                    Pc[2], 0, -Pc[0], 
                                    -Pc[1], Pc[0], 0], dtype = float).reshape(3,3)

            deriv_zt_wrt_Pc = (1/Zc) * np.array([[1, 0, -u1],
                                                [0, 1, -v1]], dtype = float)
            
            deriv_Pc_wrt_p = -R.T

            deriv_zt_wrt_p = deriv_zt_wrt_Pc @ deriv_Pc_wrt_p 
            deriv_zt_wrt_theta = deriv_zt_wrt_Pc  @ deriv_Pc_wrt_theta 

            return deriv_zt_wrt_p, deriv_zt_wrt_theta

        
        deriv_zt_wrt_p, deriv_zt_wrt_theta = get_derivatives()

        H_t = np.zeros((2, 18))
        H_t[:, 0:3] = deriv_zt_wrt_p
        H_t[:, 6:9] = deriv_zt_wrt_theta

        K_t = Sigma_old @ H_t.T @ np.linalg.inv((H_t @ Sigma_old @ H_t.T) + Q_t)
        delta_x = K_t @ innovation
        delta_x = delta_x.reshape((-1,1))

        p += delta_x[0:3]
        v += delta_x[3:6]
        q *= Rotation.from_rotvec((delta_x[6:9]).reshape(-1,))

        a_b += delta_x[9:12]
        w_b += delta_x[12:15]
        g += delta_x[15:18]

        error_state_covariance = (I - K_t @ H_t) @ Sigma_old @ (I - K_t @ H_t).T + K_t @ Q_t @ K_t.T

        return (p, v, q, a_b, w_b, g), error_state_covariance, innovation

