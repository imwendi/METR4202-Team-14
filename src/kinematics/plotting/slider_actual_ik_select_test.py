import sys
from matplotlib.widgets import Slider, Button
from kinematics.kinematics import RobotKinematics
from plotter import *
from joint_controller.definitions import *

CONFIG = [182.2, 200.0, 61.8, 99.1]

base_height = BASE_HEIGHT
link_lengths = LINK_LENGTHS
rk = RobotKinematics(base_height, link_lengths)

# setup plot axes
fig = plt.figure(figsize=(10, 10))
ax = plt.axes(projection='3d')
plt.subplots_adjust(bottom=0.35)

# sliders
ax_x = plt.axes([0.25, 0.2, 0.65, 0.03])
ax_y = plt.axes([0.25, 0.15, 0.65, 0.03])
ax_z = plt.axes([0.25, 0.1, 0.65, 0.03])
ax_phi = plt.axes([0.25, 0.05, 0.65, 0.03])

lim = L + base_height
x_slide = Slider(ax_x, 'End-affector x', -lim, lim, CONFIG[0])
y_slide = Slider(ax_y, 'End-affector y', -lim, lim, CONFIG[1])
z_slide = Slider(ax_z, 'End-affector z', 0, lim, CONFIG[2])
phi_slide = Slider(ax_phi, 'orientation', -180, 180, CONFIG[3])

JOINT_ANGLES_IDX = 3
ik_solution = np.zeros((4, 1))


def update(val):
    global rk, ik_solution, last_plot, ax
    x = x_slide.val
    y = y_slide.val
    z = z_slide.val
    phi = np.deg2rad(phi_slide.val)

    ik_solution = rk.ik(np.array([x, y, z]), phi)
    ik_solution = rk.filter_ik_solution(ik_solution)
    joint_angles = rk.pick_highest_joint_2(ik_solution)

    if joint_angles is None:
        print("No valid joint angles")
    else:
        joint_pos = rk.joint_pos(joint_angles)
        # get collision
        if rk.check_self_collision(joint_angles,
                                   verbose=False,
                                   skip_consecutive=False):
            print("Collision!!!", file=sys.stderr)
        else:
            print("No collision.")

        ax.clear()
        ax.set_xlim([-lim, lim])
        ax.set_ylim([-lim, lim])
        ax.set_zlim([-lim, lim])
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')
        last_plot = plot_4r_robot(joint_pos, ax, surface_range=(-lim, lim))


# slider reset button
# Create a `matplotlib.widgets.Button` to reset the sliders to initial values.
resetax = fig.add_axes([0.8, 0.01, 0.1, 0.04])
button2 = Button(resetax, 'Reset', hovercolor='0.975')
def reset(event):
    pass
    x_slide.reset()
    y_slide.reset()
    z_slide.reset()
    phi_slide.reset()
button2.on_clicked(reset)


if __name__ == '__main__':
    update(None)
    x_slide.on_changed(update)
    y_slide.on_changed(update)
    z_slide.on_changed(update)
    phi_slide.on_changed(update)


    plt.show()
