import numpy as np
import matplotlib.pyplot as plt
from kinematics.collision import intersect_2_segments, intersect_connected_segments

segments = [np.array([0., 0.]),
            np.array([1., 0.]),
            np.array([2., 0.]),
            np.array([2., 1.]),
            np.array([1.53052844, 0.11705241]),
            np.array([2.52625314, 0.02468182])]

print(intersect_connected_segments(segments))

segments = np.array(segments)
print(segments.shape)

print(intersect_2_segments(segments[2], segments[-2], segments[3], segments[-1]))
print(segments[2], segments[-2], segments[3], segments[-1])

plt.plot(segments[:, 0], segments[:, 1])
plt.show()
