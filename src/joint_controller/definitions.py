'''
Various constants / default values

'''
import numpy as np

# robot link lengths [mm]
L1 = 46
L2 = 55
L3 = 120
L4 = 95
DEFAULT_LINK_LENGTHS = np.array([L1, L2, L3, L4])

# common node names
NODE_JOINT_STATES = 'joint_states'
NODE_DESIRED_JOINT_STATES = 'desired_joint_states'
NODE_DESIRED_POS = 'desired_pose'
