import numpy as np
from geometry_msgs.msg import Point


def numpify(msg):
    """
    Convert various ROS message classes to numpy

    Args:
        msg: message

    Returns: numpyified message

    """
    if isinstance(msg, Point):
        msg = np.array([msg.x, msg.y, msg.z])
    else:
        msg = np.array(msg)

    return msg