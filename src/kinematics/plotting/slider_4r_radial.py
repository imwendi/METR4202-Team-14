import sys
from matplotlib.widgets import Slider, Button
from kinematics.kinematics import RobotKinematics
from kinematics.path_planning import get_radial_point
from plotter import *

base_height = 0.5
link_lengths = np.array([1, 1, 1, 1])
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

lim = 3.999999

r_slide = Slider(ax_r, 'Radius', 0, lim, 1.7)
theta_slide = Slider(ax_theta, 'Direction', -np.pi, np.pi, 0)
z_slide = Slider(ax_z, 'Height', 0, lim, 0.7)
phi_slide = Slider(ax_phi, 'Orientation', -np.pi, np.pi, np.pi*3/4)

joint_angles = np.zeros((4, 1))
def update(val):
    global rk, joint_angles, last_plot, ax
    x, y, z = get_radial_point(r_slide.val, theta_slide.val, z_slide.val)
    phi = phi_slide.val

    joint_angles = rk.ik(np.array([x, y, z]), phi)
    for i in range(4):
        # check for collisions
        _joint_angles = joint_angles[:, i]
        if not rk.check_self_collision(_joint_angles):
            joint_pos = rk.joint_pos(_joint_angles)
            for _joint_pos in joint_pos:
                if _joint_pos[-1] <= z:
                    continue
            joint_angles = _joint_angles
            break

    #print(joint_angles)
    joint_pos = rk.joint_pos(joint_angles)

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
    last_plot = plot_4r_robot(joint_pos, ax, surface_range=(-lim, lim))


# slider reset button
# Create a `matplotlib.widgets.Button` to reset the sliders to initial values.
resetax = fig.add_axes([0.8, 0.025, 0.1, 0.04])
button = Button(resetax, 'Reset', hovercolor='0.975')
def reset(event):
    pass
    r_slide.reset()
    theta_slide.reset()
    z_slide.reset()
    phi_slide.reset()
button.on_clicked(reset)


if __name__ == '__main__':
    update(None)
    r_slide.on_changed(update)
    theta_slide.on_changed(update)
    z_slide.on_changed(update)
    phi_slide.on_changed(update)


    plt.show()