import rclpy, sys, time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import matplotlib.pyplot as plt

class PID():
    def __init__(self, timer_period):
        self.prevpt1 = [100,100]
        self.prevpt2 = [700,100]
        self.timer_period = timer_period
        self.error = [0,0]
        self.curr_time = 0
        
    def calc_error(self, img):
        b1 = 232
        g1 = 0
        r1 = 0
        b2 = 255
        g2 = 255
        r2 = 255
        h_min = (g1, b1, r1)
        h_max = (g2, b2, r2)

        gray = cv2.inRange(img, h_min, h_max)

        dst = gray[int(gray.shape[0]/3*2):, :]
        
        cv2.imshow("dst", dst)
        cv2.waitKey(1)
        
        dst = np.array(dst, dtype=np.int8)
        
        cnt, labels, stats, centroids = cv2.connectedComponentsWithStats(dst)

        if cnt > 1:
            mindistance1 = []
            mindistance2 = []
            for i in range(1, cnt):
                p = centroids[i]
                ptdistance = [abs(p[0] - self.prevpt1[0]), abs(p[0] - self.prevpt2[0])]
                mindistance1.append(ptdistance[0])
                mindistance2.append(ptdistance[1])

            threshdistance = [min(mindistance1), min(mindistance2)]
            minlb = [mindistance1.index(min(mindistance1)), mindistance2.index(min(mindistance2))]
            cpt = [centroids[minlb[0] + 1], centroids[minlb[1] + 1]]

            if threshdistance[0] > 50:
                cpt[0] = self.prevpt1
            if threshdistance[1] > 50:
                cpt[1] = self.prevpt2

        else:
            cpt = [self.prevpt1, self.prevpt2]

        self.prevpt1 = cpt[0]
        self.prevpt2 = cpt[1]

        fpt = [(cpt[0][0] + cpt[1][0]) / 2, (cpt[0][1] + cpt[1][1]) / 2 + gray.shape[0] / 3 * 2]
        
        self.error.append(dst.shape[1] / 2 - fpt[0])
    	
        
    def update_error(self):
        self.curr_time += self.timer_period
        Kp, Ki, Kd = (0.018, 0.0, 0.017)
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.15
        cmd_vel.angular.z = float( \
        	            Kp * self.error[-1] + \
                            Ki * np.sum(np.array(self.error)*self.timer_period) + \
                            Kd * (self.error[-1] - self.error[-2]) / self.timer_period )
        return cmd_vel
        
