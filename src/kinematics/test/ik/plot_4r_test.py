import numpy as np
import matplotlib.pyplot as plt
from kinematics.kinematics_base import KinematicsBase
from kinematics.plotting.plotter import plot_4r_robot

rk = KinematicsBase([1, 1, 1, 1])

pose = np.deg2rad(np.array([0, 0, 0, 0]))
joint_positions = rk.joint_pos([0, 0, 0, 0])

ax = plt.axes(projection='3d')
lim = 3.99999
ax.set_xlim([-lim, lim])
ax.set_ylim([-lim, lim])
ax.set_zlim([-lim, lim])

plot_4r_robot(joint_positions=joint_positions, ax=ax)
plt.show()

