import numpy as np
import modern_robotics as mr


def apply_transform(T, p):
    """
    Applies transformation matrix to point

    Args:
        T: transformation matrix
        p: point

    Returns:
        Transformed point

    """
    if (len(p.shape)) < 2:
        # reshape (n,) to (n, 1) if needed
        p = p.reshape((len(p), 1))

    # append ones row
    p = np.concatenate([p, np.ones((1, p.shape[-1]))], axis=0)

    return (T @ p)[:-1, :]


def rot(roll, pitch, yaw):
    """
    Create rotation matrix

    Args:
        roll: roll angle
        pitch: pitch angle
        yaw: yaw angle

    Returns:
        Rotation matrix in SO3

    """
    # precompute trig for efficiency
    sr, cr = np.sin(roll), np.cos(roll)
    sp, cp = np.sin(pitch), np.cos(pitch)
    sy, cy = np.sin(yaw), np.cos(yaw)

    return np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp, cp*sr, cp*cr]
    ])


def transform(pitch, roll, yaw, dx, dy, dz):
    """
    Create transformation matrix

    Args:
        pitch: pitch angle
        roll: roll angle
        yaw: yaw angle
        dx: x translation
        dy: y translation
        dz: z translation

    Returns:
        Transformation matrix in SE3

    """

    return mr.RpToTrans(rot(pitch, roll, yaw), np.array([dx, dy, dz]))
