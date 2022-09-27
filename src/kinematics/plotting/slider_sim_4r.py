from matplotlib.widgets import Slider
from kinematics.kinematics import RobotKinematics
from plotter import *


link_lengths = np.array([1, 1, 1, 1])
rk = RobotKinematics(link_lengths)

# setup plot axes
plt.figure(figsize=(10, 10))
ax = plt.axes(projection='3d')
plt.subplots_adjust(bottom=0.35)

# sliders
ax_x = plt.axes([0.25, 0.2, 0.65, 0.03])
ax_y = plt.axes([0.25, 0.15, 0.65, 0.03])
ax_z = plt.axes([0.25, 0.1, 0.65, 0.03])
ax_phi = plt.axes([0.25, 0.05, 0.65, 0.03])


lim = 3.999999

x_slide = Slider(ax_x, 'End-affector x', 0, lim, 0)
y_slide = Slider(ax_y, 'End-affector y', 0, lim, 0)
z_slide = Slider(ax_z, 'End-affector z', 0, lim, 3.999)
phi_slide = Slider(ax_phi, 'orientation', 0, 2*np.pi, 0)

joint_angles = np.zeros((4, 1))


def update(val):
    global rk, joint_angles, last_plot, ax
    x = x_slide.val
    y = y_slide.val
    z = z_slide.val
    phi = phi_slide.val

    joint_angles = rk.ik_closest_norm(np.array([x, y, z]), phi, joint_angles)
    joint_pos = rk.joint_pos(joint_angles)

    ax.clear()
    lim = 4
    ax.set_xlim([-lim, lim])
    ax.set_ylim([-lim, lim])
    ax.set_zlim([-lim, lim])
    last_plot = plot_4r_robot(joint_pos, ax)


update(None)
x_slide.on_changed(update)
y_slide.on_changed(update)
z_slide.on_changed(update)
phi_slide.on_changed(update)


plt.show()