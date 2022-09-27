import numpy as np

pairs = [[1, np.sqrt(3)],
         [np.sqrt(3), 1]]

for (a, b) in pairs:
    print("a=%.3f, b=%.3f, atan2(a, b)=%.3f, atan(a/b)=%.3f" % (a,
                                                           b,
                                                           np.rad2deg(np.arctan2(a, b)),
                                                           np.rad2deg(np.arctan(a/b))))
