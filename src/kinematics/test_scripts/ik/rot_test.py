import numpy as np
import modern_robotics as mr
from kinematics.utils import rot

l1 = 1
theta1 = np.pi/60

T1 = mr.RpToTrans(rot(0, 0, theta1), np.array([0, 0, l1]))
T2 = mr.RpToTrans(rot(0, -np.pi / 2, 0) @ rot(-np.pi/2, 0, 0), np.zeros(3))
T3 = T2@T1

T4 = mr.RpToTrans(rot(-np.pi/2, -np.pi/2, theta1), np.array([0, 0, l1]))

print(np.around(T3, 3))
print(np.around(T4, 3))
