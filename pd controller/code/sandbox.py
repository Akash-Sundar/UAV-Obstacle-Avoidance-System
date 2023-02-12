"""
This file is not used for grading at all, and you should modify it any way you find useful.
"""

import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt

from flightsim.animate import animate
from flightsim.simulate import Quadrotor, simulate
from flightsim.world import World
from flightsim.axes3ds import Axes3Ds
from flightsim.crazyflie_params import quad_params
from flightsim import hover_traj

import waypoint_traj
import se3_control

# This object defines the quadrotor dynamical model and should not be changed.
quadrotor = Quadrotor(quad_params)

# You will complete the implementation of the SE3Control object.
my_se3_control = se3_control.SE3Control(quad_params)

# This simple hover trajectory is useful for tuning control gains.
my_traj = hover_traj.HoverTraj()

# You will complete the implementation of the WaypointTraj object. It should
# work for any list of 3D coordinates, such as this example:
# points = np.array([
#     [0, 0, 0],
#     [1, 0, 0],
#     [1, 1, 0],
#     [1, 1, 1]])
# my_traj = waypoint_traj.WaypointTraj(points)

# Set simulation parameters.
#
# You may use the initial condition and a simple hover trajectory to examine the
# step response of your controller to an initial disturbance in position or
# orientation.

w = 2
world = World.empty((-w, w, -w, w, -w, w))
t_final = 60
initial_state = {'x': np.array([0, 0, 0]),
                 'v': np.zeros(3,),
                 'q': np.array([0, 0, 0, 1]), # [i,j,k,w]
                 'w': np.zeros(3,)}

# Perform simulation.
#
# This function performs the numerical simulation.  It returns arrays reporting
# the quadrotor state, the control outputs calculated by your controller, and
# the flat outputs calculated by you trajectory.

print('Simulate.')
(time, state, control, flat, exit) = simulate(initial_state,
                                              quadrotor,
                                              my_se3_control,
                                              my_traj,
                                              t_final)
print(exit.value)

# Plot Results
#
# You will need to make plots to debug your controllers and tune your gains.
# Here are some example of plots that may be useful.

# Position and Velocity vs. Time
(fig, axes) = plt.subplots(nrows=2, ncols=1, sharex=True, num='Position vs Time')
x = state['x']
x_des = flat['x']
ax = axes[0]
ax.plot(time, x_des[:,0], 'r', time, x_des[:,1], 'g', time, x_des[:,2], 'b')
ax.plot(time, x[:,0], 'r.',    time, x[:,1], 'g.',    time, x[:,2], 'b.')
ax.legend(('x', 'y', 'z'))
ax.set_ylabel('position, m')
ax.grid('major')
ax.set_title('Position')
v = state['v']
v_des = flat['x_dot']
ax = axes[1]
ax.plot(time, v_des[:,0], 'r', time, v_des[:,1], 'g', time, v_des[:,2], 'b')
ax.plot(time, v[:,0], 'r.',    time, v[:,1], 'g.',    time, v[:,2], 'b.')
ax.legend(('x', 'y', 'z'))
ax.set_ylabel('velocity, m/s')
ax.set_xlabel('time, s')
ax.grid('major')

# Orientation and Angular Velocity vs. Time
(fig, axes) = plt.subplots(nrows=2, ncols=1, sharex=True, num='Orientation vs Time')
q_des = control['cmd_q']
q = state['q']
ax = axes[0]
ax.plot(time, q_des[:,0], 'r', time, q_des[:,1], 'g', time, q_des[:,2], 'b', time, q_des[:,3], 'k')
ax.plot(time, q[:,0], 'r.',    time, q[:,1], 'g.',    time, q[:,2], 'b.',    time, q[:,3],     'k.')
ax.legend(('i', 'j', 'k', 'w'))
ax.set_ylabel('quaternion')
ax.set_xlabel('time, s')
ax.grid('major')
w = state['w']
ax = axes[1]
ax.plot(time, w[:,0], 'r.', time, w[:,1], 'g.', time, w[:,2], 'b.')
ax.legend(('x', 'y', 'z'))
ax.set_ylabel('angular velocity, rad/s')
ax.set_xlabel('time, s')
ax.grid('major')

# Commands vs. Time
(fig, axes) = plt.subplots(nrows=3, ncols=1, sharex=True, num='Commands vs Time')
s = control['cmd_motor_speeds']
ax = axes[0]
ax.plot(time, s[:,0], 'r.', time, s[:,1], 'g.', time, s[:,2], 'b.', time, s[:,3], 'k.')
ax.legend(('1', '2', '3', '4'))
ax.set_ylabel('motor speeds, rad/s')
ax.grid('major')
ax.set_title('Commands')
M = control['cmd_moment']
ax = axes[1]
ax.plot(time, M[:,0], 'r.', time, M[:,1], 'g.', time, M[:,2], 'b.')
ax.legend(('x', 'y', 'z'))
ax.set_ylabel('moment, N*m')
ax.grid('major')
T = control['cmd_thrust']
ax = axes[2]
ax.plot(time, T, 'k.')
ax.set_ylabel('thrust, N')
ax.set_xlabel('time, s')
ax.grid('major')

# 3D Paths
fig = plt.figure('3D Path')
ax = Axes3Ds(fig)
world.draw(ax)
ax.plot3D(state['x'][:,0], state['x'][:,1], state['x'][:,2], 'b.')
ax.plot3D(flat['x'][:,0], flat['x'][:,1], flat['x'][:,2], 'k')

# Animation (Slow)
# Instead of viewing the animation live, you may provide a .mp4 filename to save.
R = Rotation.from_quat(state['q']).as_matrix()


ani = animate(time, state['x'], R, world=world, filename=None)


plt.show()
