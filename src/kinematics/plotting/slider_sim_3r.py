import sys

from matplotlib.widgets import Slider, Button
from kinematics.kinematics_base import KinematicsBase
from kinematics.collision import intersect_connected_segments
from plotter import *

base_height = 1
link_lengths = np.array([1, 1, 1, 1])
rk = KinematicsBase(1, link_lengths)

# setup plot axes
fig, ax = plt.subplots()
plt.subplots_adjust(bottom=0.35)

# sliders
ax_x = plt.axes([0.25, 0.2, 0.65, 0.03])
ax_y = plt.axes([0.25, 0.15, 0.65, 0.03])
ax_phi = plt.axes([0.25, 0.1, 0.65, 0.03])
lim = 2.999999
x_slide = Slider(ax_x, 'End-affector x', 0, lim, 0)
y_slide = Slider(ax_y, 'End-affector y', -lim, lim, 0)
phi_slide = Slider(ax_phi, 'orientation', -np.pi, np.pi, 0)

def update(val):
    global possible_joint_angles, last_plot, ax
    x = x_slide.val
    y = y_slide.val
    phi = phi_slide.val

    new_angles = rk.ik_3r(link_lengths[1:], [x, y, phi])
    # get joint_angles closest to existing
    angle_idx = 0
    joint_angles = new_angles[:, angle_idx]
    joint_pos = rk.joint_pos_3r(link_lengths[1:], joint_angles)

    # get collision
    if intersect_connected_segments(joint_pos):
        print("Collision!!!", file=sys.stderr)
    else:
        print("\n")

    ax.clear()
    lim = 4
    ax.set_xlim([-lim, lim])
    ax.set_ylim([-lim, lim])
    last_plot = plot_3r_robot(joint_pos, ax)

# slider reset button
# Create a `matplotlib.widgets.Button` to reset the sliders to initial values.
resetax = fig.add_axes([0.8, 0.025, 0.1, 0.04])
button = Button(resetax, 'Reset', hovercolor='0.975')
def reset(event):
    pass
    x_slide.reset()
    y_slide.reset()
    phi_slide.reset()
button.on_clicked(reset)

possible_joint_angles = np.zeros((3, 1))


if __name__ == '__main__':
    update(None)
    x_slide.on_changed(update)
    y_slide.on_changed(update)
    phi_slide.on_changed(update)

    plt.show()