import numpy as np

# camera frame to robot base frame
T_ROBOT_CAM = np.array([[0, 1, 0, 210],
                        [1, 0, 0, 0],
                        [0, 0, -1, 426],
                        [0, 0, 0, 1]])

# T_ROBOT_CAM = np.array([[0, 1, 0, 230],
#                         [1, 0, 0, 0],
#                         [0, 0, -1, 426],
#                         [0, 0, 0, 1]])


# scale from camera to metric units
# CAMERA_SCALE = 450 / 2.42 # 185.9
CAMERA_SCALE = 30 / 0.14 # 214.285

# RGB color node
NODE_RGB_COLOR = 'test_color'

RECENT_INTERVAL = 1.5
CUBE_TIMEOUT = 3.0

# colors
RED = np.array([136, 58, 28])
GREEN = np.array([58, 97, 38])
BLUE = np.array([30, 61, 57])
YELLOW = np.array([143, 130, 45])
