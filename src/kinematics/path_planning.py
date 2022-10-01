import numpy as np
from kinematics.utils import polar2r


def get_radial_point(r, theta, height):
    return np.array([*polar2r(r, theta), height])

