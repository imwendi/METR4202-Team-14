import numpy as np

# z-height to go to above cube before attempting to grab it
FOLLOW_HEIGHT = 80
# z-height to grab cube at
GRAB_HEIGHT = 45

# position at which to check colour
COLOR_CHECK_POS = np.array([180, -10, 330])
# block dump position
DUMP_POS = np.array([210, -180, 200])

# robot starting position
START_POS = np.array([0, -150, 100])

# time within which to try grab cube from last turntable stop [s]
GRAB_CUBE_DURATION = 7

# turntable center position
TURNTABLE_CENTER = np.array([190, 0, 0])
# y displacement adjustment threshold
Y_ADJUST_THRESHOLD = 40

# closeness threshold between two positions
CLOSENESS_THRESHOLD = 10

# drop-off zones
DROP_HEIGHT = 20
ZONE_1 = np.array([10, 150, DROP_HEIGHT])
# center of zone 2 (-150, 50, DROP_HEIGHT) is not reachable by the robot, so
# we use the below alternate
ZONE_2 = np.array([-135, 80, DROP_HEIGHT])
ZONE_3 = np.array([-135, -80, DROP_HEIGHT])
ZONE_4 = np.array([10, -150, DROP_HEIGHT])

# color to zone map
COLOR_ZONES = {
    'blue': ZONE_1,
    'yellow': ZONE_2,
    'green': ZONE_3,
    'red': ZONE_4
}

LEFT_HOME = np.array([100, 130, FOLLOW_HEIGHT])
RIGHT_HOME = np.array([100, -130, FOLLOW_HEIGHT])

# color node
NODE_COLOR = 'detected_color'