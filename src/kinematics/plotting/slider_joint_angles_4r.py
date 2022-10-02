import sys
from matplotlib.widgets import Slider, Button
from kinematics.kinematics import RobotKinematics
from plotter import *

base_height = 1.0
link_lengths = np.array([1, 1, 1, 1])
rk = RobotKinematics(base_height, link_lengths)

# setup plot axes
fig = plt.figure(figsize=(10, 10))
ax = plt.axes(projection='3d')
plt.subplots_adjust(bottom=0.35)

# sliders
ax0 = plt.axes([0.25, 0.2, 0.65, 0.03])
ax1 = plt.axes([0.25, 0.15, 0.65, 0.03])
ax2 = plt.axes([0.25, 0.1, 0.65, 0.03])
ax3 = plt.axes([0.25, 0.05, 0.65, 0.03])

lim = 3.999999

slide_0 = Slider(ax0, 'joint angle 0', -180, 180, 0)
slide_1 = Slider(ax1, 'joint angle 1', -180, 180, 90)
slide_2 = Slider(ax2, 'joint angle 3', -180, 180, 152)
slide_3 = Slider(ax3, 'joint angle 4', -180, 180, 112.7)

possible_joint_angles = np.zeros((4, 1))
sliders = [slide_0, slide_1, slide_2, slide_3]

def update(val):
    global rk, possible_joint_angles, last_plot, ax
    joint_angles = [np.deg2rad(slider.val) for slider in sliders]
    joint_pos = rk.joint_pos(joint_angles)

    # get collision
    if rk.check_self_collision(joint_angles, verbose=True):
        print("Collision!!!", file=sys.stderr)
    else:
        print("no collision")
    print('\n')

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
resetax = fig.add_axes([0.8, 0.01, 0.1, 0.04])
button = Button(resetax, 'Reset', hovercolor='0.975')
def reset(event):
    pass
    for slider in sliders:
        slider.reset()
button.on_clicked(reset)


if __name__ == '__main__':
    update(None)
    for slider in sliders:
        slider.on_changed(update)

    plt.show()
    