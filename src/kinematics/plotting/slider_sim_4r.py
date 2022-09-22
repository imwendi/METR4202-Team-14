from matplotlib.widgets import Slider
from kinematics.kinematics import RobotKinematics
from plotter import *


link_lengths = np.array([1, 1, 1, 1])
rk = RobotKinematics(link_lengths)

# setup plot axes
fig, ax = plt.subplots()
plt.subplots_adjust(bottom=0.35)

# sliders
ax_x = plt.axes([0.25, 0.2, 0.65, 0.03])
ax_y = plt.axes([0.25, 0.15, 0.65, 0.03])
ax_z = plt.axes([0.25, 0.1, 0.65, 0.03])
ax_phi = plt.axes([0.25, 0.05, 0.65, 0.03])


lim = 2.999999

x_slide = Slider(ax_x, 'End-affector x', 0, lim, 0)
y_slide = Slider(ax_y, 'End-affector y', 0, lim, 0)
z_slide = Slider(ax_z, 'End-affector z', 0, lim, 0)
phi_slide = Slider(ax_phi, 'orientation', 0, 2*np.pi, 0)

joint_angles = np.zeros((3, 1))

def update(val):
    global joint_angles, last_plot, ax
    x = x_slide.val
    y = y_slide.val
    z = z_slide.val
    phi = phi_slide.val

    new_angles = rk.ik_3r(link_lengths[1:], [x, y, phi])
    # get joint_angles closest to existing
    norm_val = np.linalg.norm(new_angles - joint_angles.reshape((3, 1)))
    angle_idx = np.argmin(norm_val, axis=-1)
    joint_angles = new_angles[:, angle_idx]
    joint_pos = rk.joint_pos_3r(link_lengths[1:], joint_angles)

    ax.clear()
    lim = 4
    ax.set_xlim([-lim, lim])
    ax.set_ylim([-lim, lim])
    last_plot = plot_3r_robot(joint_pos, ax)

update(None)
x_slide.on_changed(update)
y_slide.on_changed(update)
phi_slide.on_changed(update)


plt.show()