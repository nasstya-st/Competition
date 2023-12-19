import rclpy, sys
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import os, time

class CirclePublisher(Node):

    def __init__(self):
        super().__init__('publisher')
        self.subscription = self.create_subscription(Image, "/color/image",
               self.image_callback, 10)
        self.i = 0

    def image_callback(self, msg):
        
        directory = '/home/nastya/ros2_ws/src/stepanova_anastasia_autorace_core/dataset'
        os.chdir(directory)
        cv_bridge = CvBridge()
        frame = cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        new_filename = str(self.i)+'_image.png'
        cv2.imwrite(new_filename, frame)
        self.i+=1
        time.sleep(10)



def main(args=None):
    rclpy.init(args=args)

    circling = CirclePublisher()

    rclpy.spin(circling)

    #circling.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
