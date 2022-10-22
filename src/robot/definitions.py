import numpy as np

# z-height to go to above cube before attempting to grab it
FOLLOW_HEIGHT = 75
# z-height to grab cube at
GRAB_HEIGHT = 40
# position at which to check colour
COLOR_CHECK_POS = np.array([210, -20, 300])
# block dump position
DUMP_POS = np.array([210, -180, 200])
# robot starting position
START_POS = np.array([0, -150, 100])

# threshold for if two positions (i.e. claw and cube) are "close"
CLOSENESS_THRESHOLD = 10

# time out for moving to a cube
TIMEOUT = 2

# drop-off zones
DROP_HEIGHT = 40
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

# color node
NODE_COLOR = 'detected_color'