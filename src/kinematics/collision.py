import copy
from typing import List

import numpy as np


def intersect_connected_segments(points=List[np.array],
                                 verbose=True,
                                 skip_consecutive=False):
    """
    Determines if a series of consecutive line segments (i.e. robot arm)
    have any self-collision.

    Args:
        points: joint positions [a1, a2, a3, ..., an]
        skip_consecutive: set true to skip collision checking between
                          consecutive line segments

    Returns:
        If any two line segments between (ai and ai+1), (aj and aj+1)
        intersect.

    """
    # list conversion is necessary, otherwise double iteration over zip doesn't work
    line_segments = list(zip(points[:-1], points[1:]))

    # TODO: do we need this?
    num_segments = len(line_segments)
    for i in range(num_segments):
        a1, b1 = line_segments[i]
        # skip consecutive
        for j in range(i + (2 if skip_consecutive else 1), num_segments):
            a2, b2 = line_segments[j]

            if intersect_2_segments(a1, a2, b1, b2):
                # break and return on intersection found
                if verbose:
                    print(f"collision between ({a1} -> {b1}) and ({a2} -> {b2})")

                return True

    return False


def intersect_2_segments(a1, a2, b1, b2):
    """
    Single computation for if two 2d line segments intersect

    Args:
        a1: line segment 1 start point
        a2: line segment 2 start point
        b1: line segment 1 end point
        b2: line segment 2 end point

    Returns:
        true if intersecting, else false

    """
    # end result
    intersecting = False

    # 2d vector cross product
    cross = lambda v, w: v[0]*w[1] - v[1]*w[0]
    # re-positioned lines
    v1, v2 = b1 - a1, b2 - a2
    # precompute m1 x m2
    v1_cross_v2 = cross(v1, v2)

    # check if lines are parallel and co-linear
    parallel = v1_cross_v2 == 0
    colinear = parallel and cross(a2 - a1, v1) == 0

    if parallel:
        if colinear:
            # TODO: check this!!
            # check if co-linear lines intersect
            t1 = np.dot(a2 - a1, v1) / np.dot(v1, v1)
            t2 = t1 + np.dot(v2, v1) / np.dot(v1, v1)
            print(t2 - t1)
            intersecting = 0 < np.abs(t2 - t1) < 1
        # else parallel, not co-linear cannot intersect

        return intersecting

    # not parallel, so at most a single point of intersection
    # scaled distances along each segment to intersection
    t1 = cross(a2 - a1, v2) / v1_cross_v2
    t2 = cross(a2 - a1, v1) / v1_cross_v2

    # no intersection if either of t1, t2 are NaN or not in range (0, 1)
    if not np.isnan(t1) and not np.isnan(t2) and 0 < t1 < 1 and 0 < t2 < 1:
        intersecting = True

    return intersecting

