from typing import Union
import numpy as np
import matplotlib.pyplot as plt


def plot_4r_robot(joint_positions,
                  ax: plt.Axes,
                  surface_range: Union[None, tuple] = None):
    """
    Plots the 2R robot for a given configuration.

    Args:
        joint_positions: joint positions
        ax: plot axis
        surface_range: given a tuple (-lim, lim), plots a surface with
        (-lim, lim) as its x, y axes limits
    """
    # Plot links
    joint_positions = np.stack(joint_positions)
    x, y, z = joint_positions[0, :], joint_positions[1, :], joint_positions[2, :]

    # Plot links
    ax.plot3D(x, y, z, color='black')
    # Plot joints
    ax.scatter(x[:-1], y[:-1], z[:-1], color='black', edgecolor=None)
    # End-affector in red
    ax.scatter(x[-1], y[-1], z[-1], color='red', edgecolor=None, zorder=42069)

    # plot base
    ax.plot3D([0, x[0]], [0, y[0]], [0, z[0]], linewidth=5)

    # plot surface
    if surface_range is not None:
        X, Y = np.meshgrid(np.array(surface_range), np.array(surface_range))
        Z = np.zeros(X.shape)
        ax.plot_surface(X, Y, Z,
                        antialiased=False,
                        color='green',
                        alpha=0.1)

def plot_3r_robot(joint_positions, ax: plt.Axes, end_text=True):
    """
    Plots the 2R robot for a given configuration.

    Args:
        joint_positions: joint positions
        end_text: set true to label end-affector position
        ax: plot axis
    """
    # Plot links
    joint_positions = np.stack(joint_positions)
    x, y = joint_positions[:, 0], joint_positions[:, 1]

    # Plot links
    ax.plot(x, y, color='black')
    # Plot joints
    ax.scatter(x[:-1], y[:-1], color='black', edgecolor=None)
    # End-affector in red
    end_x, end_y = x[-1], y[-1]
    ax.scatter(end_x, end_y, color='red', edgecolor=None, zorder=42069)
    if end_text:
        ax.text(end_x, end_y, '(%.2f, %.2f)' % (end_x, end_y))