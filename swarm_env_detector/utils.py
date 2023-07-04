import cv2
import numpy as np
import threading
from scipy.spatial.transform import Rotation
from swarm_env_detector.config import *
from swarm_env_detector.classes import *

def process_frame(frame):
        markerCorners, markerIds, _ = cv2.aruco.detectMarkers(frame, A_DICT, parameters=A_PARAMS)
        cv2.aruco.drawDetectedMarkers(frame, markerCorners, markerIds)
        cv2.imshow("frame", frame)
        if markerIds is None:
            return
        navigation_markers, drone_markers = [], []
        
        for index, id in enumerate(markerIds):
            id = id[0]
            
            if id in NAVIGATION_DICT:
                navigation_markers.append((id, markerCorners[index]))
            if id in DRONES_DICT:
                drone_markers.append((id, markerCorners[index]))
        
        abs_t, yaw = get_abs_location(navigation_markers)
        drones_loc = get_drones_location(drone_markers)


def get_abs_location(location_markers):
    """
    get abs average location from mapped markers

    Args:
        location_markers (list[tuple(int, tuple(int))]): list of ids and corosponding corners

    Returns:
        tuple(T_vec,float): average T vec and yaw from origin
    """
    avrg_T = []
    avrg_yaw = 0
    for i in location_markers:
        id, corners = i
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, NAVIGATION_DICT[id].SIZE, CAMERA_MAT, CAMERA_DIST)
        R, T = extract_6_dof(rvecs[0][0], tvecs[0][0])
        a_T, a_yaw = NAVIGATION_DICT[id].get_abs_location(T, R.yaw)
        avrg_T.append(a_T)
        avrg_yaw += a_yaw
    
    return T_vec.avrg(avrg_T), (avrg_yaw/len(location_markers))%360

def get_drones_location(drones_markers):
    """
    gets all the locations of drones. each drone only once

    Args:
        drones_markers (tuple(int, tuple(int))): ids and marker corners of drone markers

    Returns:
        list[T_vecs]: a list of drone locations
    """
    
    seen_ids, drone_locations = [],[]
    for i in drones_markers:
        id, corners = i
        if id in seen_ids:
            continue
        seen_ids.append(id)
        
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, NAVIGATION_DICT[id].SIZE, CAMERA_MAT, CAMERA_DIST)
        R, T = extract_6_dof(rvecs[0][0], tvecs[0][0], origin_marker=False)
        drone_locations.append(T)
    return T


def extract_6_dof(rvec, tvec, origin_marker = True, round_num = None):
    """
    extract 6 dof from vecs

    Args:
        rvec (list(int)): rotation rodrigez vec
        tvec (list(int)): translation vec in mm
        origin_marker (bool, optional): sets the return origin - marker or origin. Defaults to True.
        round_num (_type_, optional): number of digits to round return to. Defaults to None.

    Returns:
        tuple(R_vec, T_vec): rotation and translation in degrees and cm
    """
    rmat, _ = cv2.Rodrigues(rvec)
    rmat = np.matrix(rmat)
    euler_angles = Rotation.from_matrix(rmat).as_euler("xyz", degrees=True)
    
    if origin_marker:
        tvec = (tvec * -rmat)
        tvec = np.array(tvec[0])[0]
    
    pitch = euler_angles[0]
    if pitch > 0: 
        pitch = 180-pitch
    else:
        pitch = -pitch - 180

    yaw = euler_angles[1]
    roll = euler_angles[2]
    
    R = R_vec(pitch, roll, yaw)
    T = T_vec(tvec[0]/10, tvec[2]/10, tvec[1]/10)
    
    if round_num != None:
        R = R_vec(round(R.pitch, round_num), round(R.roll, round_num), round(R.yaw, round_num))
        T = T_vec(round(T.x, round_num), round(T.y, round_num), round(T.z, round_num))
    
    return R, T


def test():
    fs = cv2.FileStorage("/home/fares/rbd/tools/calib/webcam.yaml", cv2.FILE_STORAGE_READ)
    CAMERA_MAT = fs.getNode("camera_matrix").mat()
    CAMERA_DIST = fs.getNode("distortion_coefficients").mat()
    print (CAMERA_MAT)
    print (CAMERA_DIST)
    
    cap = cv2.VideoCapture(0)
    while cv2.waitKey(50) & 0xFF != ord('q'):
        ret, frame = cap.read()
        if not ret:
            continue
        
        markerCorners, markerIds, _ = cv2.aruco.detectMarkers(frame, A_DICT, parameters=A_PARAMS)
        cv2.aruco.drawDetectedMarkers(frame, markerCorners, markerIds)
        cv2.imshow("frame", frame)
        if markerIds is None:
            continue
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(markerCorners, 70, CAMERA_MAT, CAMERA_DIST)
        r,t = extract_6_dof(rvecs[0][0], tvecs[0][0], origin_marker=False, round_num=1)
        print (r, t)
