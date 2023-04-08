#%% Imports

import numpy as np
from scipy.spatial.transform import Rotation
from matplotlib import pyplot as plt
import yaml
import stereo
from vio import *
from numpy.linalg import norm

# %% Import IMU dataset
#  CSV imu file
dirname = '../dataset/MachineHall01_reduced/imu0/'

imu0 = np.genfromtxt(dirname + 'data.csv', delimiter=',', dtype='float64', skip_header=1)

# pull out components of data set - different views of matrix

# timestamps in nanoseconds
imu_timestamp = imu0[:, 0]

# angular velocities in radians per second
angular_velocity = imu0[:, 1:4]

# linear acceleration in meters per second^2
linear_acceleration = imu0[:, 4:]

#%% Read IMU calibration data

with open(dirname + 'sensor.yaml', 'r') as file:
    imu_calib_data = yaml.load(file, Loader=yaml.FullLoader)

gyroscope_noise_density = imu_calib_data['gyroscope_noise_density']
gyroscope_random_walk = imu_calib_data['gyroscope_random_walk']
accelerometer_noise_density = imu_calib_data['accelerometer_noise_density']
accelerometer_random_walk = imu_calib_data['accelerometer_random_walk']

# %% Import stereo dataset
main_data_dir = "../dataset/MachineHall01_reduced/"
dataset = stereo.StereoDataSet(main_data_dir)

# Extract rotation that transforms IMU to left camera frame
R_LB = dataset.stereo_calibration.tr_base_left[0:3, 0:3].T

# %% Initialize filter

imu_index = 0
stereo_index = 0

first_image_timestamp = float(dataset.get_timestamp(0))

while imu_timestamp[imu_index] < first_image_timestamp:
    imu_index += 1

stereo_pair_2 = dataset.process_stereo_pair(0)
stereo_index += 1

next_image_time = float(dataset.get_timestamp(stereo_index))

last_timestamp = first_image_timestamp

nimages = 200

focal_length = dataset.rectified_camera_matrix[0, 0]

image_measurement_covariance = ((0.5 / focal_length) ** 2) * np.eye(2)
error_threshold = (10 / focal_length)

# Initialize state
p = np.zeros((3, 1))
v = np.zeros((3, 1))
q = Rotation.identity()
a_b = np.zeros((3, 1))
w_b = np.zeros((3, 1))

g = R_LB @ linear_acceleration[imu_index]
g *= (-9.8 / norm(g))
g = g.reshape(3, 1)

nominal_state = p, v, q, a_b, w_b, g

# Initialize error state covariance
error_state_covariance = np.diag([0, 0, 0, 0.2, 0.2, 0.2, 0.1, 0.1, 0.1, 0.25, 0.25, 0.25, 0.02, 0.02, 0.02, 0, 0, 0])

# These variables encode last stereo pose
last_R = nominal_state[2].as_matrix()
last_t = nominal_state[0]

trace_covariance = []
pose = []

# %% Main Loop
while True:

    if imu_index >= imu0.shape[0]:
        break

    if stereo_index >= nimages:
        break

    trace_covariance.append(error_state_covariance.trace())
    pose.append((nominal_state[2], nominal_state[0], nominal_state[1], nominal_state[3].copy()))

    # Extract prevailing a_m and w_m - transform to left camera frame
    w_m = R_LB @ angular_velocity[imu_index - 1, :].reshape(3, 1)
    a_m = R_LB @ linear_acceleration[imu_index - 1, :].reshape(3, 1)

    t = min(imu_timestamp[imu_index], next_image_time)
    dt = (t - last_timestamp) * 1e-9
    last_timestamp = t

    # Apply IMU update
    error_state_covariance = error_covariance_update(nominal_state, error_state_covariance, w_m, a_m, dt,
                                                     accelerometer_noise_density, gyroscope_noise_density,
                                                     accelerometer_random_walk, gyroscope_random_walk)
    nominal_state = nominal_state_update(nominal_state, w_m, a_m, dt)

    if imu_timestamp[imu_index] <= next_image_time:
        # IMU update
        imu_index += 1
    else:
        # Stereo update
        stereo_pair_1 = stereo_pair_2
        stereo_pair_2 = dataset.process_stereo_pair(stereo_index)

        stereo_index += 1
        next_image_time = float(dataset.get_timestamp(stereo_index))

        temporal_match = stereo.TemporalMatch(stereo_pair_1, stereo_pair_2)

        uvd1, uvd2 = temporal_match.get_normalized_matches(dataset.rectified_camera_matrix, dataset.stereo_baseline)

        innovations = np.zeros((2, uvd1.shape[1]))

        for i in range(0, uvd1.shape[1]):
            # Compute Pw
            u1, v1, d1 = uvd1[:, i]

            if d1 > 0:
                P1 = np.array([u1 / d1, v1 / d1, 1 / d1]).reshape(3, 1)

                Pw = last_R @ P1 + last_t

                # Extract uv
                uv = uvd2[0:2, i].reshape(2, 1)

                nominal_state, error_state_covariance, inno = measurement_update_step(nominal_state,
                                                                                      error_state_covariance,
                                                                                      uv, Pw, error_threshold,
                                                                                      image_measurement_covariance)

                innovations[:, i] = inno.ravel()

        count = (norm(innovations, axis=0) < error_threshold).sum()

        pixel_error = np.median(abs(innovations), axis=1) * focal_length

        print("{} / {} inlier ratio, x_error {:.4f}, y_error {:.4f}, norm_v {:.4f}".format(count, uvd1.shape[1],
                                                                                           pixel_error[0],
                                                                                           pixel_error[1],
                                                                                           norm(nominal_state[1])))

        # These variables encode last stereo pose
        last_R = nominal_state[2].as_matrix()
        last_t = nominal_state[0]

# %% Gather results

n = len(pose)

euler = np.zeros((n, 3))
translation = np.zeros((n, 3))
velocity = np.zeros((n, 3))
a_bias = np.zeros((n, 3))

for (i, p) in enumerate(pose):
    euler[i] = p[0].as_euler('XYZ', degrees=True)
    translation[i] = p[1].ravel()
    velocity[i] = p[2].ravel()
    a_bias[i] = p[3].ravel()

# %% Plot trace of covariance matrix

plt.plot(trace_covariance)
plt.title('Trace of covariance matrix')
plt.show()

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

#%%

plt.figure()
plt.plot(velocity[:, 0], label='vx')
plt.plot(velocity[:, 1], label='vy')
plt.plot(velocity[:, 2], label='vz')
plt.ylabel('meters per second')
plt.title('Velocity of Quad')
plt.legend()
plt.show()

#%%
plt.figure()
plt.plot(a_bias[:, 0], label='ax')
plt.plot(a_bias[:, 1], label='ay')
plt.plot(a_bias[:, 2], label='az')
plt.ylabel('meters per second squared')
plt.title('Accelerometer Bias')
plt.legend()
plt.show()
