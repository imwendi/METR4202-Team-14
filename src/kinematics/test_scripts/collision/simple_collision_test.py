import numpy as np
import matplotlib.pyplot as plt
from kinematics.collision import intersect_2_segments


if __name__ == '__main__':
    # # 2D test (co-linear, intersects)
    # a1 = np.array([0, 0])
    # b1 = np.array([0, 5])
    # a2 = np.array([0, 3])
    # b2 = np.array([0, 8])
    # print(intersect_2_segments(a1, a2, b1, b2))
    #
    # # 2D test (co-linear, no intersection)
    # a1 = np.array([0, 0])
    # b1 = np.array([0, 2])
    # a2 = np.array([0, 3])
    # b2 = np.array([0, 8])
    # print(intersect_2_segments(a1, a2, b1, b2))
    #
    # # 2D test (expect intersection at (2.5, 2.5))
    # a1 = np.array([0, 0])
    # b1 = np.array([5, 5])
    # a2 = np.array([0, 5])
    # b2 = np.array([5, 0])
    # print(intersect_2_segments(a1, a2, b1, b2))
    #
    # # 2D test (parallel, no intersection)
    # a1 = np.array([0, 1])
    # b1 = np.array([0, 5])
    # a2 = np.array([1, 1])
    # b2 = np.array([1, 5])
    # print(intersect_2_segments(a1, a2, b1, b2))

    a1 = np.array([0, 0])
    b1 = np.array([0, 10])
    a2 = np.array([0, 10])
    b2 = np.array([0, 14])
    print(intersect_2_segments(a1, a2, b1, b2))

    # a1 = np.array([46, 0])
    # b1 = np.array([101, 0])
    # a2 = np.array([0, 0])
    # b2 = np.array([46, 0])
    # print(intersect_2_segments(a1, a2, b1, b2))
