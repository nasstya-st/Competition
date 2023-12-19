import rclpy, sys, time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, String
from sensor_msgs.msg import Image, LaserScan
from rclpy.executors import MultiThreadedExecutor
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import LaserScan
import cv2
import numpy as np
import matplotlib.pyplot as plt

from .sign_recognition import recognition

global depth_image
global not_check_first_two
not_check_first_two = 0

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
    
    def empty_listener_callback(self, msg):        
        if msg.data=='start':
            self.is_started = 1        
        if msg.data=='stop':
            self.is_started = 0
    
    def update_pose(self, ranges):
        self.scan = ranges    	
    
    def check_sign(self):
        global time_to_check
        time_to_check = 0
        global not_check_first_two
        laser = self.scan.ranges
        if len(laser)==0: return
        for i in range(20):
            if laser[5+i]<0.5 or laser[i+334]<0.5:
                if not_check_first_two == 2:
                    time_to_check = 1
                else: not_check_first_two+=1
    
    def recognizer_callback(self, msg):
        if not self.is_started: 
            return 
        cv_bridge = CvBridge()
        global timer
        global depth_image
        frame = cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        #frame[:, :int(frame.shape[1] //2)] = [0,0,0]
        #p_depth_image = np.array(depth_image, dtype=np.dtype('f8'))
        self.check_sign()
        findedClass = 'none'
        if time_to_check:
            copy_depth = np.copy(depth_image)
       	    cur_img, findedClass, names = recognition(frame, copy_depth) # сделать депт имадж глобальной переменной и создать подписчика
       	    cv2.imshow("result", cur_img)
            cv2.waitKey(1)
            self.get_logger().info(f'found: {findedClass}')
            self.get_logger().info(f'searching: {names}')
       	message = String()
       	message.data = str(findedClass)
       	self.sign_printer.publish(message)
        
        
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
