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