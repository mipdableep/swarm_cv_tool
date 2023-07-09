import cv2
from swarm_env_detector.classes import *

#                        id  size  x    y    z   yaw
m52  = navigation_marker(52, 166, 500, 0  , 60 , 180)
m53  = navigation_marker(53, 166, 850, 0  , 77 , 180)
m54  = navigation_marker(54, 166, 0  , 213, 134, 90 )
m55  = navigation_marker(55, 166, 407, 481, 62 , 0  )
m56  = navigation_marker(56, 166, 0  , 433, 109, 90 )
m57  = navigation_marker(57, 166, 158, 488, 53 , 90 )
m58  = navigation_marker(58, 166, 712, 610, 91 , 0  )
m59  = navigation_marker(59, 166, 825, 610, 93 , 0  )
m60  = navigation_marker(60, 166, 565, 50 , 61 , 270)
m61  = navigation_marker(61, 166, 855, 453, 55 , 270)
m62  = navigation_marker(62, 166, 967, 87 , 60 , 270)
m62  = navigation_marker(62, 166, 309, 0  , 137, 180)
m777 = navigation_marker(777,148, 454, 521, 115, 0  )

navigation_markers = [m52,m53,m54,m55,m56,m57,m58,m59,m60,m61,m62,m62,m777]


NAVIGATION_DICT = {i.ID:i for i in navigation_markers}
DRONES_DICT = {800:80, 801:80, 802:80,}


A_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
A_PARAMS = cv2.aruco.DetectorParameters()

