from typing import List

import numpy as np


def intersect_connected_segments(points=List[np.array]):
    """
    #TODO: pretty inefficient, very O(n^2)

    Determines if a series of consecutive line segments (i.e. robot arm)
    have any self-collision.

    Args:
        points: Joint positions [a1, a2, a3, ..., an]

    Returns:
        If any two line segments between (ai and ai+1), (aj and aj+1)
        intersect.

    """
    line_segments = zip(points[:-1], points[1:])
    for i, (a1, b1) in enumerate(line_segments):
        for j, (a2, b2) in enumerate(line_segments):
            if i == j:
                # same line
                continue

            if intersect_2_segments(a1, a2, b1, b2) is not None:
                print(f'a1:{a1}, b1:{b1}')
                print(f'a2:{a2}, b2:{b2}')
                # break and return on first valid intersection found
                return True

    return False

def intersect_2_segments(a1, a2, b1, b2):
    """
    Single computation for intersection of two line segments

    Args:
        a1: line segment 1 start point
        a2: line segment 2 start point
        b1: line segment 1 end point
        b2: line segment 2 end point

    Returns:
        returns intersection position vector, or None if it doesn't exist

    """
    intersection = None

    # print(a1, b1)
    # print(a2, b2)

    # re-positioned lines
    m1, m2 = b1 - a1, b2 - a2
    # precompute m1 x m2
    m1_cross_m2 = np.cross(m1, m2)

    # if cross product is the zero vector, the no solution
    # TODO: should we use np.isclose(..., np.zeros()) in case this value is
    # TODO: very close to zero vector, but not exactly equal to it?
    if not np.any(m1_cross_m2):
        # scaled distances along each segment to intersection
        t1 = np.cross(a2 - a1, m2) / m1_cross_m2
        t2 = np.cross(a2 - a1, m1) / m1_cross_m2

        # no intersection if either of t1, t2 are NaN or not in range (0, 1)
        if not np.isnan(t1) and not np.isnan(t2) and 0 < t1 < 1 and 0 < t2 < 1:
            intersection = a1 + t1*m1

    return intersection

