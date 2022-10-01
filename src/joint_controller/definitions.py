'''
Various constants / default values

'''
import numpy as np

# common node names
NODE_JOINT_STATES = 'joint_states'
NODE_DESIRED_JOINT_STATES = 'desired_joint_states'
NODE_DESIRED_POS = 'desired_pose'
NODE_DESIRED_POSE4 = 'desired_pose4'

# robot measurements [mm] (426 total)
BASE_HEIGHT = 46
L1 = 55
L2 = 120
L3 = 95
L4 = 110
LINK_LENGTHS = np.array([L1, L2, L3, L4])
L = np.sum(LINK_LENGTHS)

# motion
DEFAULT_VELOCITY = 0.5 # [rad/s]



if __name__ == '__main__':
    print(L)
