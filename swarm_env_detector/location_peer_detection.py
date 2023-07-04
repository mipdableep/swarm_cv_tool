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
        CAMERA_MAT = fs.getNode("camera_matrix").mat()
        CAMERA_DIST = fs.getNode("distortion_coefficients").mat()
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


def main():
    rclpy.init()
    node = swarm_detector()
    rclpy.spin(node)

if __name__ == '__main__':
    main()