'''
Various constants / default values

'''
import numpy as np

# ___________________________________ ROS ______________________________________
# common node names
NODE_JOINT_STATES = 'joint_states'
NODE_DESIRED_JOINT_STATES = 'desired_joint_states'
NODE_DESIRED_POS = 'desired_pose'
NODE_DESIRED_POSE4 = 'desired_pose4'

# _________________________________ GEOMETRY ___________________________________
# robot measurements [mm] (426 total)
BASE_HEIGHT = 46
L1 = 55
L2 = 120
L3 = 95
L4 = 110
LINK_LENGTHS = np.array([L1, L2, L3, L4])
L = np.sum(LINK_LENGTHS)

# no joints should have vertical (z) position below these value to avoid
# ground collision
VERTICAL_THRESHOLD = 10  # [mm]

# ________________________________ DYNAMIXEL ___________________________________
# Dynamixel 4 angle needs to be *= -1 as it is installed in opposite orientation
# to Dynamixels 2 and 3
DIRECTIONAL_MULTIPLIERS = np.array([1, 1, 1, -1])
DEFAULT_VELOCITY = 0.5  # [rad/s]



if __name__ == '__main__':
    print(L)
