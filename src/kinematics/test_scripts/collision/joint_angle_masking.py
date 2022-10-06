import numpy as np

MIN = np.array([-10, -10, -20, -20])
MAX = np.array([10, 20, 30, 40])

def filter(x):
    min_mask = (MIN < x)
    max_mask = (MAX > x)

    print('min mask ', min_mask)
    print('max mask ', max_mask)

    select_idx = np.logical_and(min_mask, max_mask).all(axis=0)

    print('select_idx ', select_idx)

    x = x[:, select_idx]

    return x


if __name__ == '__main__':
    x1 = np.array([[-20, 1, 1, 1],
                   [5, 5, 5, 5],
                   [420, 0, 0, -50],
                   [0, 0, 0, 0]]).T

    x1_filtered = filter(x1)
    print('x1', np.around(x1, 3))
    print('x1_filtered', np.around(x1_filtered, 3))