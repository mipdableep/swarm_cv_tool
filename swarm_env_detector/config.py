import cv2
from swarm_env_detector.classes import *

navigation_markers = list[navigation_marker]
m52 = navigation_marker(52, 166, 500, 0, 60, 180)

CAMERA_MAT, CAMERA_DIST = None, None

NAVIGATION_DICT = {i.ID:i for i in navigation_markers}
DRONES_DICT = {800:80, 801:80, 802:80,}


A_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
A_PARAMS = cv2.aruco.DetectorParameters()

