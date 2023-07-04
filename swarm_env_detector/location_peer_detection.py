import rclpy
from rclpy.node import Node
from example_interfaces.msg import MicroLocation, Drones, Point
from rcl_interfaces.msg import ParameterDescriptor

from swarm_env_detector.VCS import VideoCapture
import cv2
import threading
from time import sleep

from swarm_env_detector.classes import *
from swarm_env_detector.config import *
from swarm_env_detector.utils import *


class swarm_cv_tool(Node):
    def __init__(self):
        super().__init__('swarm_cv_tool')
        
        self._video_ip = None
        self._video_port = None
        self._calib_path = None
        self._video_cap = None
        self._publish_hrz = None
        
        self.abs_T, self.abs_yaw = None, None
        self.drones_location = None

        self.location_lock = False
        self.peers_lock = False
        
        # set parameters from ros args
        params = []
        # param video_ip
        params.append(self.declare_parameter
        (
            name="swarm_cv_tool.video_ip",
            value="0.0.0.0",
            descriptor=ParameterDescriptor(description="video ip.")
        ))
        # param video_port
        params.append(self.declare_parameter
        (
            name="swarm_cv_tool.video_port",
            value=11111,
            descriptor=ParameterDescriptor(description="video port.")
        ))
        # param calib_path
        params.append(self.declare_parameter
        (
            name="swarm_cv_tool.calib_path",
            value="/ws_ros/ws/src/swarm_env_detector/drone_calib/drone20.yaml",
            descriptor=ParameterDescriptor(description="path to calibration yaml file.")
        ))
        # param publishing hrz
        params.append(self.declare_parameter
        (
            name="swarm_cv_tool.publish_hrz",
            value=10,
            descriptor=ParameterDescriptor(description="amount of publishes in 1 sec")
        ))
        
        self.update_parameters(params)
        
        fs = cv2.FileStorage(self._calib_path, cv2.FILE_STORAGE_READ)
        CAMERA_MAT = fs.getNode("camera_matrix").mat()
        CAMERA_DIST = fs.getNode("distortion_coefficients").mat()
        self.get_logger().info(f"cam mat:{self._camera_mat}")
        self.get_logger().info(f"cam dist:{self._camera_dist}")
        
        # publishers
        self._pub_location = self.create_publisher(MicroLocation, "location", 10)
        self._pub_peer_location = self.create_publisher(Drones, "peer_detection", 10)
        
        self._timer = self.create_timer(1/self._publish_hrz, self.write_and_publish())
        
        self.runloop()

  
    def update_parameters(self, event):
        """update the class params from ros params

        Args:
            event : list of params
        """
        for p in event:
            self.get_logger().info(f'Parameter Updated: {p.name} = {p.value}')
            if (p.name == "swarm_cv_tool.video_ip"):
                self._video_ip = p.value
            if (p.name == "swarm_cv_tool.video_port"):
                self._video_port = p.value
            if (p.name == "swarm_cv_tool.calib_path"):
                self._calib_path = p.value
            if (p.name == "swarm_cv_tool.publish_hrz"):
                self._publish_hrz = p.value
    
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
            if id in DRONES_DICT:
                drone_markers.append((id, markerCorners[index]))
        if navigation_markers != []:
            abs_t, abs_yaw = get_abs_location(navigation_markers)
            # write to self vals with thread protection
            while self.location_lock:
                sleep(0.001)
            
            self.location_lock = True
            self.abs_T, self.abs_yaw = abs_t, abs_yaw
            self.location_lock = False

        if drone_markers != []:
            drones_loc = get_drones_location(drone_markers)
            # write to self vals with thread protection
            while self.peers_lock:
                sleep(0.001)
            
            self.peers_lock = True
            self.drones_location = drones_loc
            self.peers_lock = False

    def write_and_publish(self):
        if not self.location_lock and self.abs_T != None:
            self.location_lock = True
            
            loc_msg = MicroLocation()
            loc_msg.x = self.abs_T.x
            loc_msg.y = self.abs_T.y
            loc_msg.z = self.abs_T.z
            loc_msg.yaw = self.abs_yaw
            
            self.location_lock = False
            
            self._pub_location.publish(loc_msg)
        
        if not self.peers_lock and self.drones_location != None:
            self.peers_lock = True
            peers_msg = Drones()
            for i in self.drones_location:
                peers_msg.drones.append(Point(x = i.x, y = i.y, z = i.z))
            self.peers_lock = False
            
            self._pub_peer_location.publish(peers_msg)

    def runloop(self):
        self._video_cap = VideoCapture(f"udp://{self._video_ip}:{self._video_port}")
        while self._video_cap._live:
            ret, frame = self._video_cap.read()
            if not ret:
                continue
            
            process_frame(frame)

def main():
    rclpy.init()
    node = swarm_cv_tool()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
    