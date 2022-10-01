import sys
from matplotlib.widgets import Slider, Button
from kinematics.kinematics import RobotKinematics
from plotter import *


link_lengths = np.array([1, 1, 1, 1])
rk = RobotKinematics(link_lengths)

# setup plot axes
fig = plt.figure(figsize=(10, 10))
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
z_slide = Slider(ax_z, 'End-affector z', 0, lim, lim/2)
phi_slide = Slider(ax_phi, 'orientation', -np.pi, np.pi, 0)

joint_angles = np.zeros((4, 1))


def update(val):
    global rk, joint_angles, last_plot, ax
    x = x_slide.val
    y = y_slide.val
    z = z_slide.val
    phi = phi_slide.val

    joint_angles = rk.ik(np.array([x, y, z]), phi)[:, 3]
    print(joint_angles)
    joint_pos = rk.joint_pos_geom(joint_angles)

    # get collision
    if rk.check_self_collision(joint_angles):
        print("Collision!!!", file=sys.stderr)
    else:
        print("\n")

    ax.clear()
    lim = 4
    ax.set_xlim([-lim, lim])
    ax.set_ylim([-lim, lim])
    ax.set_zlim([-lim, lim])
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    last_plot = plot_4r_robot(joint_pos, ax)


# slider reset button
# Create a `matplotlib.widgets.Button` to reset the sliders to initial values.
resetax = fig.add_axes([0.8, 0.025, 0.1, 0.04])
button = Button(resetax, 'Reset', hovercolor='0.975')
def reset(event):
    pass
    x_slide.reset()
    y_slide.reset()
    z_slide.reset()
    phi_slide.reset()
button.on_clicked(reset)


if __name__ == '__main__':
    update(None)
    x_slide.on_changed(update)
    y_slide.on_changed(update)
    z_slide.on_changed(update)
    phi_slide.on_changed(update)


    plt.show()