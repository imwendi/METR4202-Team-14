import numpy as np

# z-height to go to above cube before attempting to grab it
FOLLOW_HEIGHT = 65
# z-height to grab cube at
GRAB_HEIGHT = 30
# position at which to check colour
COLOR_CHECK_POS = np.array([210, -20, 300])
# block dump position
DUMP_POS = np.array([210, -180, 200])

# drop-off zones
ZONE_1 = np.array([-50, 150])
ZONE_2 = np.array([-150, 50])
ZONE_3 = np.array([-150, -50])
ZONE_4 = np.array([-50, -150])

# color to zone map
COLOR_ZONES = {
    'BLUE': ZONE_1,
    'YELLOW': ZONE_2,
    #'GREEN': ZONE_3,
    #'RED': ZONE_4
}

# color node
NODE_COLOR = 'detected_color'