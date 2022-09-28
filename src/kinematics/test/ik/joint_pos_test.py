import numpy as np
from kinematics.kinematics import RobotKinematics

rk = RobotKinematics([1, 1, 1, 1])
pos = rk.joint_pos([0, 0, 0, 0])

print(np.around(pos, 3))

