'''
Various constants / default values

'''
import numpy as np

# common node names
NODE_JOINT_STATES = 'joint_states'
NODE_DESIRED_JOINT_STATES = 'desired_joint_states'
NODE_DESIRED_POS = 'desired_pose'

# robot link lengths [mm]
L1 = 46
L2 = 55
L3 = 120
L4 = 95
DEFAULT_LINK_LENGTHS = np.array([L1, L2, L3, L4])
L = np.sum(DEFAULT_LINK_LENGTHS)

# motion
DEFAULT_VELOCITY = 0.0001 # [m/s]



if __name__ == '__main__':
    print(L)
