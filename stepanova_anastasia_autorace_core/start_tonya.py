import rclpy, sys, time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, String
from sensor_msgs.msg import Image, LaserScan
from rclpy.executors import MultiThreadedExecutor
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import LaserScan
import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt

from .sign_recognition import lab, recognition

MIN_MATCH_COUNT = 4

def assay(good, matchesMask1, img2):
	if len(good)>MIN_MATCH_COUNT:
		src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
		dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)
		M, mask = cv.findHomography(src_pts, dst_pts, cv.RANSAC,5.0)
		matchesMask1 = mask.ravel().tolist()
		h,w = img1.shape
		pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
		dst = cv.perspectiveTransform(pts,M)
		img2 = cv.polylines(img2,[np.int32(dst)],True,255,3, cv.LINE_AA)
		return matchesMask1
	else:
		print( "Not enough matches are found - {}/{}".format(len(good), MIN_MATCH_COUNT) )
		matchesMask1 = None
		return matchesMask1

def mat(des2):
	matches = flann.knnMatch(des1,des2,k=2)
	return matches
	
def g(matches):
	good = []
	for m,n in matches:
		if m.distance < 0.8*n.distance:
			good.append(m)
	return good


def lab():
    detector = cv.SIFT_create()
    

global depth_image
global not_check_first_two
not_check_first_two = 0
path_list = get_package_share_directory('stepanova_anastasia_autorace_core')
PATH_TEMPLATE = "signes"
full_path =  os.path.join(path_list, PATH_TEMPLATE, 'traffic_left.png')

time_to_check = 0
class Recognition(Node):
    def __init__(self):
        super().__init__('subscription')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 1)
        self.sign_printer = self.create_publisher(String, '/signes', 1)

        self.subscription = self.create_subscription(String, "robot_stop", self.empty_listener_callback, 1)

        self.starter = self.create_subscription(Image, "/color/image", self.traffic_light_callback, 1)
        self.recognizer = self.create_subscription(Image, "/color/image", self.recognizer_callback, 1)
        self.depth = self.create_subscription(Image, "/depth/image", self.depth_callback, 1)
        self.pose_subscriber = self.create_subscription(LaserScan, '/scan', self.update_pose, 10)
        self.scan = LaserScan()
        self.is_started = 0
        
        self.detector = cv.SIFT_create()
    
    def empty_listener_callback(self, msg):        
        if msg.data=='start':
            self.is_started = 1        
        if msg.data=='stop':
            self.is_started = 0
    
    def update_pose(self, ranges):
        self.scan = ranges    	
    
    
    def recognizer_callback(self, msg):
        if not self.is_started: 
            return 
        cv_bridge = CvBridge()
        global timer
        global depth_image
        img1 = cv.imread('mini.png', 0)
        img2 = cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        kp1, des1 = self.detector.detectAndCompute(img1,None)
        FLANN_INDEX_KDTREE = 1
        index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
        search_params = dict(checks = 20)
        flann = cv.FlannBasedMatcher(index_params, search_params)
        m = mat(des2)
        good = g(m)
        #matchesMask = ()
        matchesMask = assay(good, matchesMask, img2)  # none - right, else - left
        if matchesMask:
            msg = String()
            msg.data = 'traffic_left'
            self.sign_printer.publish(msg)
        else:
            msg = String()
            msg.data = 'traffic_right'
            self.sign_printer.publish(msg)
    def depth_callback(self, msg):
        global depth_image
        cv_bridge = CvBridge()
        depth_image = cv_bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        

    
    def traffic_light_callback(self, msg):
        cv_bridge = CvBridge()
        frame = cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if not self.is_started:
            if frame[300, 600][1]==109: self.is_started = 1
      
      
def main(args=None):
    rclpy.init(args=args)
    recognsing= Recognition()
    rclpy.spin(recognsing)
    rclpy.shutdown()
