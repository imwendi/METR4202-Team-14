import numpy as np
import time
from kinematics.kinematics_base import KinematicsBase

p = np.array([1, 1, 1])
link_lengths = np.array([1, 1, 1])

start = time.time()
a = KinematicsBase.ik_3r(p, link_lengths)
end = time.time()
print("%.5fs elapsed" % (end - start))

print(a.shape)
print(np.rad2deg(a))