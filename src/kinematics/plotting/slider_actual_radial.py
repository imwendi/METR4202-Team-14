import sys
from matplotlib.widgets import Slider, Button
from kinematics.kinematics import RobotKinematics
from kinematics.path_planning import get_radial_point
from plotter import *
from joint_controller.definitions import *

base_height = BASE_HEIGHT
link_lengths = LINK_LENGTHS
rk = RobotKinematics(base_height, link_lengths)

# setup plot axes
fig = plt.figure(figsize=(10, 10))
ax = plt.axes(projection='3d')
plt.subplots_adjust(bottom=0.35)

# sliders
ax_r = plt.axes([0.25, 0.2, 0.65, 0.03])
ax_theta = plt.axes([0.25, 0.15, 0.65, 0.03])
ax_z = plt.axes([0.25, 0.1, 0.65, 0.03])
ax_phi = plt.axes([0.25, 0.05, 0.65, 0.03])

lim = L + base_height
r_slide = Slider(ax_r, 'Radius', 0, lim, 249)
theta_slide = Slider(ax_theta, 'Direction', -180, 180, 0)
z_slide = Slider(ax_z, 'End-affector z', 0, lim, 63)
phi_slide = Slider(ax_phi, 'orientation', -180, 180, -131)

JOINT_ANGLES_IDX = 3
ik_solution = np.zeros((4, 1))


def update(val):
    global rk, ik_solution, last_plot, ax
    r = r_slide.val
    theta = np.deg2rad(theta_slide.val)
    z = z_slide.val
    x, y, z = get_radial_point(r, theta, z)
    phi = np.deg2rad(phi_slide.val)

    possible_joint_angles = rk.ik(np.array([x, y, z]), phi)
    # uncomment to print options!!
    # for i in range(4):
    #     joint_angles = possible_joint_angles[:, i]
    #     val = np.around(rk.joint_pos(joint_angles), 3)
    #     deg_angles = np.around(np.rad2deg((joint_angles)), 3)
    #
    #     print(f"Option {i}\nPos:{val}\nJoint Angles:{deg_angles}\n")
    # print('_'*80)

    joint_angles = possible_joint_angles[:, JOINT_ANGLES_IDX]
    joint_pos = rk.joint_pos(joint_angles)

    # get collision
    if rk.check_self_collision(joint_angles):
        print("Collision!!!", file=sys.stderr)
    else:
        print("\n")

    ax.clear()
    ax.set_xlim([-lim, lim])
    ax.set_ylim([-lim, lim])
    ax.set_zlim([-lim, lim])
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    last_plot = plot_4r_robot(joint_pos, ax, surface_range=(-lim, lim))


# joint angle idx button
# Create a `matplotlib.widgets.Button` to reset the sliders to initial values.
joint_angle_idx_ax = fig.add_axes([0.4, 0.01, 0.1, 0.04])
button1 = Button(joint_angle_idx_ax, str(JOINT_ANGLES_IDX), hovercolor='0.975')
def change_joint_angle_idx(event):
    global JOINT_ANGLES_IDX
    JOINT_ANGLES_IDX = (JOINT_ANGLES_IDX + 1) % 4
    button1.label.set_text(str(JOINT_ANGLES_IDX))
    update(None)
button1.on_clicked(change_joint_angle_idx)


# slider reset button
# Create a `matplotlib.widgets.Button` to reset the sliders to initial values.
resetax = fig.add_axes([0.8, 0.01, 0.1, 0.04])
button2 = Button(resetax, 'Reset', hovercolor='0.975')
def reset(event):
    pass
    r_slide.reset()
    theta_slide.reset()
    z_slide.reset()
    phi_slide.reset()
button2.on_clicked(reset)


if __name__ == '__main__':
    update(None)
    r_slide.on_changed(update)
    theta_slide.on_changed(update)
    z_slide.on_changed(update)
    phi_slide.on_changed(update)


    plt.show()
