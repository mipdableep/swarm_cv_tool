import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from swarm_interfaces.msg import Microlocation, Peers
from rcl_interfaces.msg import ParameterDescriptor

from swarm_env_detector.VCS import VideoCapture
import cv2

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
        
        self.abs_T, self.abs_yaw = None, None
        self.drones_location = None

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
            value="/ros_ws/src/swarm_cv_tool/drone_calib/drone20.yaml",
            descriptor=ParameterDescriptor(description="path to calibration yaml file.")
        ))
        
        self.update_parameters(params)
        print (f"calib_path:{self._calib_path}, type:{type(self._calib_path)}")
        fs = cv2.FileStorage(str(self._calib_path), cv2.FILE_STORAGE_READ)
        CAMERA_MAT = fs.getNode("camera_matrix").mat()
        CAMERA_DIST = fs.getNode("distortion_coefficients").mat()
        self.get_logger().info(f"cam mat:{CAMERA_MAT}")
        self.get_logger().info(f"cam dist:{CAMERA_DIST}")
        
        # publishers
        self._pub_location = self.create_publisher(Microlocation, "location", 10)
        self._pub_peer_location = self.create_publisher(Peers, "peer_detection", 10)


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
            self.abs_T, self.abs_yaw = get_abs_location(navigation_markers)

        if drone_markers != []:
            self.drones_location = get_drones_location(drone_markers)
        
        self.write_and_publish()
        rclpy.spin_once(self)

    def write_and_publish(self):
        
        loc_msg = Microlocation()
        loc_msg.x = self.abs_T.x
        loc_msg.y = self.abs_T.y
        loc_msg.z = self.abs_T.z
        loc_msg.yaw = self.abs_yaw
        
        peers_msg = Peers()
        for i in self.drones_location:
            peers_msg.peers.append(Point(x = i.x, y = i.y, z = i.z))
        
        self._pub_location.publish(loc_msg)
        self._pub_peer_location.publish(peers_msg)

    def runloop(self):
        self._video_cap = VideoCapture(f"udp://{self._video_ip}:{self._video_port}")
        
        while cv2.waitKey(50) & 0xFF != ord('q'):
            ret, frame = self._video_cap.read()
            if not ret:
                continue
            
            process_frame(frame)

def main(args = None):
    # ros2 initialization
    rclpy.init(args=args)

    node_swarm_cv_tool = swarm_cv_tool()
    print("\n\naaaaaaaaaaaa\n\n")
    node_swarm_cv_tool.runloop()
    
    node_swarm_cv_tool.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()