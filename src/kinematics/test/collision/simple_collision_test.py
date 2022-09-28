import numpy as np
import matplotlib.pyplot as plt
from kinematics.collision import intersect_2_segments


if __name__ == '__main__':
    # 2D test (expect intersection at (2.5, 2.5))
    a1 = np.array([0, 0])
    a2 = np.array([0, 5])
    b1 = np.array([5, 5])
    b2 = np.array([5, 0])
    print(intersect_2_segments(a1, a2, b1, b2))

    # 2D test (expect none)
    # 2D test (expect intersection at (2.5, 2.5))
    a1 = np.array([0, 0])
    a2 = np.array([0, 5])
    b1 = np.array([1, 0])
    b2 = np.array([1, 5])
    print(intersect_2_segments(a1, a2, b1, b2))

    # 3D test (expect intersection at (2.5, 2.5))
    a1 = np.array([0, 0, 42069])
    a2 = np.array([0, 5, 42069])
    b1 = np.array([5, 5, 42069])
    b2 = np.array([5, 0, 42069])
    print(intersect_2_segments(a1, a2, b1, b2))