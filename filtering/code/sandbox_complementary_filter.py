# %% Imports

import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt
from complementary_filter import complementary_filter_update

# %%  CSV imu file
fname = '../dataset/MachineHall01_reduced/imu0/data.csv'

# %%
imu0 = np.genfromtxt(fname, delimiter=',', dtype='float64', skip_header=1)

# %% pull out components of data set - different views of matrix

# timestamps in nanoseconds
t = imu0[:, 0]

# angular velocities in radians per second
angular_velocity = imu0[:, 1:4]

# linear acceleration in meters per second^2
linear_acceleration = imu0[:, 4:]

# %% Process the imu data

n = imu0.shape[0]

euler = np.zeros((n, 3))

R = Rotation.identity()
for i in range(1, n):
    dt = (t[i] - t[i - 1]) * 1e-9
    R = complementary_filter_update(R, angular_velocity[i - 1], linear_acceleration[i], dt)
    euler[i] = R.as_euler('XYZ', degrees=True)

# %% Plots

t2 = (t - t[0]) * 1e-9

fig = plt.figure()
plt.plot(t2, euler[:, 0], label='yaw')
plt.plot(t2, euler[:, 1], label='pitch')
plt.plot(t2, euler[:, 2], label='roll')
plt.ylabel('degrees')
plt.xlabel('seconds')
plt.title('Attitude of Quad')
plt.legend()
plt.show()
