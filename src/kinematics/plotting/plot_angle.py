import numpy as np
import matplotlib.pyplot as plt
from joint_controller.definitions import *
from kinematics.kinematics import RobotKinematics
from plotter import plot_4r_robot

JOINT_ANGLES = np.array([1.571, 1.472, -0.441, -0.856])
rk = RobotKinematics(BASE_HEIGHT, LINK_LENGTHS)
joint_pos = rk.joint_pos(JOINT_ANGLES)
fig = plt.figure(figsize=(10, 10))
lim = 500
ax = plt.axes(projection='3d')
ax.set_xlim([-lim, lim])
ax.set_ylim([-lim, lim])
ax.set_zlim([-lim, lim])
plot_4r_robot(joint_pos, ax)

plt.show()


