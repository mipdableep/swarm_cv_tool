import rclpy
from rclpy.node import Node
from example_interfaces.msg import MicroLocation, Drones
from rcl_interfaces.msg import ParameterDescriptor

import swarm_env_detector.VCS
import cv2
import numpy as np
import threading
from scipy.spatial.transform import Rotation

from swarm_env_detector.classes import *
from swarm_env_detector.config import *


class swarm_detector(Node):
    def __init__(self):
        super().__init__('swarm_detector')
        
        self._video_ip = None
        self._video_port = None
        self._calib_path = None
        self._video_cap = None
        self._camera_mat = None
        self._camera_dist = None

        self.msg_lock = False
        
        
        # set parameters from ros args
        params = []
        # param video_ip
        params.append(self.declare_parameter
        (
            name="swarm_detector.video_ip",
            value="0.0.0.0",
            descriptor=ParameterDescriptor(description="video ip.")
        ))
        # param video_port
        params.append(self.declare_parameter
        (
            name="swarm_detector.video_port",
            value=11111,
            descriptor=ParameterDescriptor(description="video port.")
        ))
        # param calib_path
        params.append(self.declare_parameter
        (
            name="swarm_detector.calib_path",
            value="/ws_ros/ws/src/swarm_env_detector/drone_calib/drone20.yaml",
            descriptor=ParameterDescriptor(description="path to calibration yaml file.")
        ))
        
        self.update_parameters(params)
        
        fs = cv2.FileStorage(self._calib_path, cv2.FILE_STORAGE_READ)
        self._camera_mat = fs.getNode("camera_matrix").mat()
        self._camera_dist = fs.getNode("distortion_coefficients").mat()
        self.get_logger().info(f"cam mat:{self._camera_mat}")
        self.get_logger().info(f"cam dist:{self._camera_dist}")


  
    def update_parameters(self, event):
        """update the class params from ros params

        Args:
            event : list of params
        """
        for p in event:
            self.get_logger().info(f'Parameter Updated: {p.name} = {p.value}')
            if (p.name == "swarm_detector.video_ip"):
                self._video_ip = p.value
            if (p.name == "swarm_detector.video_port"):
                self._video_port = p.value
            if (p.name == "swarm_detector.calib_path"):
                self._calib_path = p.value


    def process_frame(self, frame):
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
            if id in drone_ids_sizes:
                drone_markers.append((id, markerCorners[index]))
        
        
    def get_location_from_markers(self, location_markers):
        avrg_R, avrg_T = [],[]
        for i in location_markers:
            id, corners = i
            NAVIGATION_DICT[id] = m50
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners = corners,
                markerLength = NAVIGATION_DICT[id].SIZE,
                cameraMatrix = self._camera_mat,
                distCoeffs = self._camera_dist)
            rvec, tvec = rvecs[0][0], tvecs[0][0]
            R, T = self.extract_6_dof_from_vecs(rvec, tvec)
            avrg_R.append(R_vec.subtract(R, NAVIGATION_DICT[id].A_R))
            avrg_T.append(T_vec.subtract(T, NAVIGATION_DICT[id].A_T))
        
        return R_vec.avrg(avrg_R), T_vec.avrg(avrg_T)

    @staticmethod
    def extract_6_dof_from_vecs(rvec, tvec)-> tuple[R_vec, T_vec]:
        """extract dof from r and t

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




def main():
    rclpy.init()
    node = swarm_detector()
    rclpy.spin(node)

if __name__ == '__main__':
    main()