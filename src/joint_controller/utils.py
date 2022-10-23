import numpy as np
from geometry_msgs.msg import Point, Vector3, Quaternion


def numpify(msg):
    """
    Convert various ROS message classes to numpy

    Args:
        msg: message

    Returns: numpyified message

    """
    if isinstance(msg, Point) or isinstance(msg, Vector3):
        msg = np.array([msg.x, msg.y, msg.z])
    elif isinstance(msg, Quaternion):
        msg = np.array([msg.x, msg.y, msg.z, msg.w])
    else:
        msg = np.array(msg)

    return msg


def quintic_timescale(ts=2):
    """
    Computes 5th order time-scaling polynomial for finish time ts.
    Originally written for METR4202 Problem Set 2 (Wendi)

    Args:
        ts: finish time

    Returns: polynomial coefficients

    """
    system = np.array([[ts**5, ts**4, ts**3],
                       [5*ts**4, 4*ts**3, 3*ts**2],
                       [20*ts**3, 12*ts**2, 6*ts]])

    coeffs = np.append(np.linalg.solve(system, np.array([1, 0, 0])), np.zeros(3))

    return coeffs
