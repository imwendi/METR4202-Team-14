import numpy as np
from kinematics.kinematics import RobotKinematics

print(np.array(RobotKinematics.joint_pos_3r([1, 1, 1], [0, 0, 0])))

