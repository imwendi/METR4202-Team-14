import numpy as np

# camera frame to robot base frame
T_ROBOT_CAM = np.array([[0, 1, 0, 230],
                        [1, 0, 0, 0],
                        [0, 0, -1, 426],
                        [0, 0, 0, 1]])

# T_ROBOT_CAM = np.array([[0, 1, 0, 190],
#                         [1, 0, 0, 0],
#                         [0, 0, -1, 426],
#                         [0, 0, 0, 1]])

# scale from camera to metric units
CAMERA_SCALE = 450 / 2.42

# RGB color node
NODE_RGB_COLOR = 'test_color'

RECENT_INTERVAL = 1.5
CUBE_TIMEOUT = 1.0

# colors
RED = np.array([255, 0, 0])
GREEN = np.array([0, 255, 0])
BLUE = np.array([0, 0, 255])
YELLOW = np.array([255, 255, 0])
