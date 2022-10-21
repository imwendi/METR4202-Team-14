import numpy as np

# z-height to go to above cube before attempting to grab it
FOLLOW_HEIGHT = 65
# z-height to grab cube at
GRAB_HEIGHT = 30
# position at which to check colour
COLOR_CHECK_POS = np.array([210, -20, 300])
# block dump position
DUMP_POS = np.array([210, -180, 200])
# default home position
HOME_POS = np.array([0, 0, 425.5])

# drop-off zones
DROP_HEIGHT = 30
ZONE_1 = np.array([-50, 150, DROP_HEIGHT])
ZONE_2 = np.array([-150, 50, DROP_HEIGHT])
ZONE_3 = np.array([-150, -50, DROP_HEIGHT])
ZONE_4 = np.array([-50, -150, DROP_HEIGHT])

# color to zone map
COLOR_ZONES = {
    'blue': ZONE_3,
    'yellow': ZONE_4,
    #'green': ZONE_3,
    #'red': ZONE_4
}

# color node
NODE_COLOR = 'detected_color'