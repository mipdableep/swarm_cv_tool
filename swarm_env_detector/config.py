import cv2
from swarm_env_detector.classes import *

m50 = navigation_marker(50,100)

navigation_markers = [m50]

NAVIGATION_DICT = {i.id:i for i in navigation_markers}
drone_ids_sizes = {800:80, 801:80, 802:80,}


A_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
A_PARAMS = cv2.aruco.DetectorParameters()

