import numpy as np
import matplotlib.colors as colors


def hsv_distance(color1, color2):
    """
    Converts RGB colors to HSV and computes the distance between them in
    cartesian coords.

    Args:
        color1: Unnormalized RGB color 1 (0-255)
        color2: Unnormalized RGB color 2 (0-255)

    Returns:
        cartesian distance in HSV space

    """
    color1_cart = hsv_to_cart(rgb_to_hsv(color1))
    color2_cart = hsv_to_cart(rgb_to_hsv(color2))

    return np.linalg.norm(color1_cart - color2_cart)


def rgb_to_hsv(rgb):
    """
    RGB (0-255) to normalized (0-1) HSV

    Args:
        rgb: RGB val

    Returns:
        normalized HSV value

    """
    # normalize
    rgb = rgb / 255.0

    return colors.rgb_to_hsv(rgb)


def hsv_to_cart(hsv):
    """
    HSV to cartesian conversion
    Based on code from
    https://github.com/UQ-METR4202/metr4202_ximea_ros/blob/main/ximea_color/src/ximea_color_detect.cpp

    Args:
        hsv: normalized HSV color

    Returns:

    """
    h = np.deg2rad(hsv[0])
    s = hsv[1]
    v = hsv[2]

    x = v * s * np.cos(h)
    y = v * s * np.sin(h)
    z = v

    return np.array([x, y, z])
