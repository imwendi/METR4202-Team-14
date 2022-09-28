import numpy as np
import time
from kinematics.kinematics import RobotKinematics

p = np.array([0, 0, 3.99])
phi = 0
link_lengths = np.array([1, 1, 1, 1])

start = time.time()
rk = RobotKinematics(link_lengths)
a = rk.ik_closest_norm(p, phi, np.array([0, 0, 0, 0]))
end = time.time()
print("%.5fs elapsed" % (end - start))

print(a)