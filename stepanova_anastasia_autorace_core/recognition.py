import rclpy, sys, time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from sensor_msgs.msg import Image, LaserScan
from rclpy.executors import MultiThreadedExecutor
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import matplotlib.pyplot as plt

from .sign_recognition import lab, recognition


class Recognition(Node):
    def __init__(self):
        super().__init__('subscription')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 1)
        self.sign_printer = self.create_publisher(String, '/signes', 1)
        self.subscription = self.create_subscription(Empty, "robot_start", self.empty_listener_callback, 5)
        self.starter = self.create_subscription(Image, "/color/image", self.traffic_light_callback, 1)
        self.recognizer = self.create_subscription(Image, "/color/image", self.recognizer_callback, 1)
        self.is_started = 0
    	
    
    def recognizer_callback(self, msg):
        if not self.is_started: 
            return 
        cv_bridge = CvBridge()
        global timer
        frame = cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
       	cur_img, findedClass, names = recognition(frame)
        cv2.imshow("result", cur_img)
        cv2.waitKey(1)
        self.get_logger().info(f'found: {findedClass}')
        self.get_logger().info(f'searching: {names}')
        
        
    def traffic_light_callback(self, msg):
        cv_bridge = CvBridge()
        frame = cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if not self.is_started:
            if frame[300, 600][1]==109: self.is_started = 1
            
    def empty_listener_callback(self, msg):
        self.is_started = 1 
      
def main(args=None):
    rclpy.init(args=args)
    recognsing= Recognition()
    rclpy.spin(recognsing)
    rclpy.shutdown()

