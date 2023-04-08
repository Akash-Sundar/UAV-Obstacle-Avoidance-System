# Imports

import numpy as np
from scipy.spatial.transform import Rotation


# %%

def estimate_pose(uvd1, uvd2, pose_iterations, ransac_iterations, ransac_threshold):
    """
    Estimate Pose by repeatedly calling ransac

    :param uvd1:
    :param uvd2:
    :param pose_iterations:
    :param ransac_iterations:
    :param ransac_threshold:
    :return: Rotation, R; Translation, T; inliers, array of n booleans
    """

    R = Rotation.identity()

    for i in range(0, pose_iterations):
        w, t, inliers = ransac_pose(uvd1, uvd2, R, ransac_iterations, ransac_threshold)
        R = Rotation.from_rotvec(w.ravel()) * R

    return R, t, inliers

def solve_w_t(uvd1, uvd2, R0):
    """
    solve_w_t core routine used to compute best fit w and t given a set of stereo correspondences

    :param uvd1: 3xn ndarray : normailzed stereo results from frame 1
    :param uvd2: 3xn ndarray : normailzed stereo results from frame 1
    :param R0: Rotation type - base rotation estimate
    :return: w, t : 3x1 ndarray estimate for rotation vector, 3x1 ndarray estimate for translation
    """

    # TODO 
    w = t = np.zeros((3,1))
    n = uvd1.shape[1]

    R0 = R0.as_matrix()
    u1, v1, d1 = uvd1[0], uvd1[1], uvd1[2]
    u2, v2, d2 = uvd2[0], uvd2[1], uvd2[2]

    point = np.array([u2[0], v2[0], 1]).reshape(-1,1)

    y = R0 @ point

    Q = np.array([[1, 0, -u1[0]], 
                  [0, 1, -v1[0]]], dtype=float)
    
    wt = np.array(
        [[0, y[2,0], -y[1,0], d2[0], 0, 0],
         [-y[2,0], 0, y[0,0], 0, d2[0], 0],
         [y[1,0], -y[0,0], 0, 0, 0, d2[0]]], dtype=float)
    
    A = Q @ wt    
    B = -Q @ y

    for i in range(1, n):
        point = np.array([u2[i], v2[i], 1]).reshape(-1,1)
        y = R0 @ point

        q = np.array([[1, 0, -u1[i]], 
                      [0, 1, -v1[i]]], dtype=float)
        
        wt = np.array(
        [[0, y[2,0], -y[1,0], d2[i], 0, 0],
         [-y[2,0], 0, y[0,0], 0, d2[i], 0],
         [y[1,0], -y[0,0], 0, 0, 0, d2[i]]], dtype=float)
        
        a = q @ wt      
        b = -q @ y

        A = np.vstack((A, a))
        B = np.vstack((B, b))

    x = np.linalg.lstsq(A, B, rcond=None)

    w,t = x[0][:3], x[0][3:]

    return w, t


def find_inliers(w, t, uvd1, uvd2, R0, threshold):
    """

    find_inliers core routine used to detect which correspondences are inliers

    :param w: ndarray with 3 entries angular velocity vector in radians/sec
    :param t: ndarray with 3 entries, translation vector
    :param uvd1: 3xn ndarray : normailzed stereo results from frame 1
    :param uvd2:  3xn ndarray : normailzed stereo results from frame 2
    :param R0: Rotation type - base rotation estimate
    :param threshold: Threshold to use
    :return: ndarray with n boolean entries : Only True for correspondences that pass the test
    """


    n = uvd1.shape[1]

    # TODO 

    u1, v1, d1 = uvd1[0], uvd1[1], uvd1[2]
    u2, v2, d2 = uvd2[0], uvd2[1], uvd2[2]

    R0 = R0.as_matrix()

    inliers = np.zeros(n, dtype='bool')

    for i in range(n):
        point = np.array([u2[i], v2[i], 1]).reshape(-1,1)
        y = R0 @ point

        a = np.array([[1, 0, -u1[i]], 
                      [0, 1, -v1[i]]], dtype=float)
        
        b = np.array([[1, -w[2], w[1]], 
                      [w[2], 1, -w[0]], 
                      [-w[1], w[0], 1]], dtype=float) @ y
        
        c = d2[i] * t.reshape(-1, 1)

        dist = np.linalg.norm(a @ (b+c))

        if dist < threshold:
            inliers[i] = True
        else:
            inliers[i] = False

    return inliers


def ransac_pose(uvd1, uvd2, R0, ransac_iterations, ransac_threshold):
    """

    ransac_pose routine used to estimate pose from stereo correspondences

    :param uvd1: 3xn ndarray : normailzed stereo results from frame 1
    :param uvd2: 3xn ndarray : normailzed stereo results from frame 1
    :param R0: Rotation type - base rotation estimate
    :param ransac_iterations: Number of RANSAC iterations to perform
    :ransac_threshold: Threshold to apply to determine correspondence inliers
    :return: w, t : 3x1 ndarray estimate for rotation vector, 3x1 ndarray estimate for translation
    :return: ndarray with n boolean entries : Only True for correspondences that are inliers

    """
    n = uvd1.shape[1]

    # TODO 
    w = t = np.zeros((3,1))
    inliers = np.zeros(n, dtype='bool')

    w_best = t_best = np.zeros((3, 1))
    max_inliers = -1    

    if ransac_iterations == 0:
        w_best, t_best = solve_w_t(uvd1, uvd2, R0)
        best_inliers = find_inliers(w_best, t_best, uvd1, uvd2, R0, ransac_threshold)
    else:
        for i in range(0, ransac_iterations):
            
            rand_idx = np.random.choice(3, n)
            uvd1_batch = uvd1[rand_idx]
            uvd2_batch = uvd2[rand_idx]
            
            w, t = solve_w_t(uvd1_batch, uvd2_batch, R0)            
            inliers = find_inliers(w, t, uvd1, uvd2, R0, ransac_threshold)
            
            num_inliers = np.count_nonzero(inliers)

            if num_inliers > max_inliers:                
                best_inliers = inliers
                max_inliers = num_inliers

                w_best, t_best = w, t

            # if max_inliers > n/2: break 

    return w_best, t_best, best_inliers
