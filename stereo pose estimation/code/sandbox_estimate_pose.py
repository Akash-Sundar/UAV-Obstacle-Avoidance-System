# %% Imports

import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt
from estimate_pose_ransac import estimate_pose
import stereo

#%% Read in dataset

main_data_dir = "../dataset/MachineHall01_reduced/"
dataset = stereo.StereoDataSet(main_data_dir)

# %% Process dataset

# Decide number of stereo pairs to process
n = 200

# Uncomment the line below to run the entire  dataset
# n  = dataset.number_of_frames

focal_length = dataset.rectified_camera_matrix[0,0]

pose_iterations = 3
ransac_threshold = 3/focal_length
ransac_iterations = 10

R = Rotation.identity()
T = np.zeros((3, 1))

pose = [(R, T)]

stereo_pair_2 = dataset.process_stereo_pair(0)

outlier_count = np.zeros(n)
inlier_count = np.zeros(n)

for index in range(1, n):
    stereo_pair_1 = stereo_pair_2
    stereo_pair_2 = dataset.process_stereo_pair(index)

    temporal_match = stereo.TemporalMatch(stereo_pair_1, stereo_pair_2)

    uvd1, uvd2 = temporal_match.get_normalized_matches(dataset.rectified_camera_matrix, dataset.stereo_baseline)

    R2,  T2, inliers = estimate_pose(uvd1, uvd2, pose_iterations, ransac_iterations, ransac_threshold)

    # update pose
    R = R * R2
    T = (R.as_matrix() @  T2) + T

    # record pose
    pose.append((R, T))

    # record inliers and outliers
    inlier_count[index] = inliers.sum();
    outlier_count[index] = inliers.size - inliers.sum()

    print(index)


#%% Gather results

euler = np.zeros((n, 3))
translation = np.zeros((n, 3))

for (i, p) in enumerate(pose):
    euler[i] = p[0].as_euler('XYZ', degrees=True)
    translation[i] = p[1].ravel()

# %% Plot results

fig = plt.figure()

plt.subplot(121)
plt.plot(euler[:, 0], label='yaw')
plt.plot(euler[:, 1], label='pitch')
plt.plot(euler[:, 2], label='roll')
plt.ylabel('degrees')
plt.title('Attitude of Quad')
plt.legend()

plt.subplot(122)
plt.plot(translation[:, 0], label='Tx')
plt.plot(translation[:, 1], label='Ty')
plt.plot(translation[:, 2], label='Tz')
plt.ylabel('meters')
plt.title('Position of Quad')
plt.legend()

plt.show()

fig = plt.figure()

plt.plot(inlier_count, label='inliers')
plt.plot(outlier_count, label='outliers')
plt.title('Inlier and outlier count')
plt.legend()

plt.show()