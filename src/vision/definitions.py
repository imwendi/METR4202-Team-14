import numpy as np

# camera frame to robot base frame
T_ROBOT_CAM = np.array([[0, 1, 0, 220],
                        [1, 0, 0, 0],
                        [0, 0, -1, 426],
                        [0, 0, 0, 1]])

# T_ROBOT_CAM = np.array([[0, 1, 0, 190],
#                         [1, 0, 0, 0],
#                         [0, 0, -1, 426],
#                         [0, 0, 0, 1]])

# scale from camera to metric units
CAMERA_SCALE = 450 / 2.42
