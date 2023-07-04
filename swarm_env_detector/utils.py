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
            # if id in DRONES_DICT:
            #     drone_markers.append((id, markerCorners[index]))
        
        t, yaw = get_location_from_markers(navigation_markers)
        print(f"abs: {t}, yaw:{yaw}")

def get_location_from_markers(location_markers) -> tuple[T_vec, float]:
    avrg_T = []
    avrg_yaw = 0
    for i in location_markers:
        id, corners = i
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, NAVIGATION_DICT[id].SIZE, CAMERA_MAT, CAMERA_DIST)
        R, T = extract_6_dof_from_vecs(rvecs[0][0], tvecs[0][0])
        a_T, a_yaw = NAVIGATION_DICT[id].get_abs_location(T, R.yaw)
        avrg_T.append(a_T)
        avrg_yaw += a_yaw
    
    return T_vec.avrg(avrg_T), (avrg_yaw/len(location_markers))%360

def extract_6_dof_from_vecs(rvec, tvec) -> tuple[R_vec, T_vec]:
    """
    extract dof from r and t

    Args:
        rvec (list): rotation vector 
        tvec (list): translation vector 

    Returns:
        tuple(R_vec, T_vec): roatation and translation in degrees and cm
    """
    rmat, _ = cv2.Rodrigues(rvec)
    rmat = np.matrix(rmat)
    euler_angles = Rotation.from_matrix(rmat).as_euler("xyz", degrees=True)
    
    tvec = (tvec * -rmat)
    tvec = np.array(tvec[0])
    
    pitch = euler_angles[0]
    if pitch > 0: 
        pitch = 180-pitch
    else:
        pitch = -pitch - 180

    yaw = euler_angles[1]
    roll = euler_angles[2]
    R = R_vec(pitch, roll, yaw)
    T = T_vec(tvec[0][0]/10, tvec[0][2]/10, tvec[0][1]/10)
    
    return R, T



if __name__ == "__main__":
    # test utils with tello
    import djitellopy
    import cv2
    import VCS
    
    # consts - change
    ip = "192.168.0.20"
    vid_port = 11111
    state_port = 8890
    cap_str = f"udp://{ip}:{vid_port}"
    waitkey_ms = 100
    
    t = djitellopy.Tello(ip)
    t.connect()
    t.set_network_ports(state_port, vid_port)
    t.streamon()
    
    cap = VCS.VideoCapture(cap_str)

    while cv2.waitKey(waitkey_ms) & 0xFF != ord('q'):
        
        ret, frame = cap.read()
        if not ret:
            continue
        process_frame(frame)

