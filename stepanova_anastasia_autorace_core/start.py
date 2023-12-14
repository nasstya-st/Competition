import rclpy, sys, time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from sensor_msgs.msg import Image, LaserScan

from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import matplotlib.pyplot as plt

from .sign_recognition import lab, recognition

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
        

class Starter(Node):

    def __init__(self):
        super().__init__('publisher')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 1)
        self.subscription = self.create_subscription(Empty, "robot_start",
         	    self.empty_listener_callback, 5)
        self.subscription = self.create_subscription(Image, "/color/image_projected_compensated",
         	    self.image_callback, 1) 	    
        self.starter = self.create_subscription(Image, "/color/image",
         	    self.traffic_light_callback, 1) 
        self.recognizer = self.create_subscription(Image, "/color/image", self.recognizer_callback, 1) 
        self.lidar = self.create_subscription(LaserScan, "/scan", self.lidar_callback, 1)  	    
        
         	    
        self.timer_period = 0.2
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.pid = PID(self.timer_period)
        
        self.lab_data = lab()
        #self.get_logger.info(f'{self.lab_data[-1]}')
        
        self.is_started = 0
        #states = ['none', 'pedestrian', 'avoid_blocks', 'parking']
        self.state = 'none'
        self.avoid_blocks_state = 0  # 0 before and after mission
        self.state_parking = 0
        self.park = 0
        self.d_r_l = 0
        
    def change_avoid_blocks_state(self, state):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.
        cmd_vel.angular.z = 0.
        self.publisher.publish(cmd_vel)
        self.avoid_blocks_state = state
        
        
    def avoid_blocks(self, data):
    	cmd_vel = Twist()
    	match self.avoid_blocks_state:
            case 0:
                if data[80] > 0.3:
                    cmd_vel.linear.x = 0.5
                    cmd_vel.angular.z = 0.
                    self.publisher.publish(cmd_vel)
                else:
                    self.change_avoid_blocks_state(2)    	
            case 1:
                if data[80] < 0.4:
                    cmd_vel.linear.x = 0.3
                    cmd_vel.angular.z = 0.
                    self.publisher.publish(cmd_vel)
                else:
                    self.change_avoid_blocks_state(2)
            case 2:
                if data[0] > 0.8:
                    cmd_vel.linear.x = 0.
                    cmd_vel.angular.z = 0.55
                    self.publisher.publish(cmd_vel)
                else:
                    self.change_avoid_blocks_state(3)
            case 3:
                if data[0] > 0.25:
                    cmd_vel.linear.x = 0.155
                    cmd_vel.angular.z = 0.
                    self.publisher.publish(cmd_vel)
                else:
                    self.change_avoid_blocks_state(4)
            case 4:
                if data[270] > 0.25:
                    cmd_vel.linear.x = 0.
                    cmd_vel.angular.z = 0.45
                    self.publisher.publish(cmd_vel)
                else:
                    self.change_avoid_blocks_state(5)    
            case 5:
                if data[285] < 0.25:
                    cmd_vel.linear.x = 0.2
                    cmd_vel.angular.z = 0.
                    self.publisher.publish(cmd_vel)
                else:
                    self.change_avoid_blocks_state(6)     
            case 6:
                if data[0] > 0.64:
                    cmd_vel.linear.x = 0.
                    cmd_vel.angular.z = -0.45
                    self.publisher.publish(cmd_vel)
                else:
                    self.change_avoid_blocks_state(7)      
            case 7:
                if data[0] > 0.35:
                    cmd_vel.linear.x = 0.2
                    cmd_vel.angular.z = 0.
                    self.publisher.publish(cmd_vel)
                else:
                    self.change_avoid_blocks_state(8)                      
            case 8:
                if data[90] > 0.4:
                    cmd_vel.linear.x = 0.
                    cmd_vel.angular.z = -0.4
                    self.publisher.publish(cmd_vel)
                else:
                    self.change_avoid_blocks_state(9)                     
            case 9:
                if data[90] < 0.4:
                    cmd_vel.linear.x = 0.2
                    cmd_vel.angular.z = 0.
                    self.publisher.publish(cmd_vel)
                else:                       
                    self.change_avoid_blocks_state(10)    
            case 10:
                if data[180] > 0.19:
                    cmd_vel.linear.x = 0.
                    cmd_vel.angular.z = 0.3
                    self.publisher.publish(cmd_vel)
                else:
                    self.change_avoid_blocks_state(-1)                  
                    self.state = 'none'  
                    
    def parking(self, ranges):
        vel_msg = Twist()
        laser = ranges
        if (len(laser)!=0):
        if(self.state_parking==0):
            if (laser[79]<0.41 or laser[281]<0.41):
               
                vel_msg.linear.x = 0.0
                self.state_parking = 1
            else:

                vel_msg.linear.x = 0.35
                
            vel_msg.angular.z = 0.0
            self.publisher.publish(vel_msg)
            if (self.state_parking==1):
                if (laser[79]<0.41):
                    self.d_r_l = laser[79]
                    self.state_parking = 2
                    self.park = 1
                if (laser[281]<0.41):
                    self.d_r_l = laser[281]
                    self.state_parking = 3
                    self.park = 2
               
                time.sleep(2)
        if (self.state_parking==2 or self.state_parking ==3):
            if self.state_parking==2:
                vel_msg.angular.z = -0.4
            if self.state_parking==3:
                vel_msg.angular.z = 0.4
            self.publisher.publish(vel_msg)
           
            if (laser[179]<0.25):
                self.d = laser[179]
                vel_msg.linear.x = 0.0
                vel_msg.angular.z = 0.0
                self.publisher.publish(vel_msg)
                self.state_parking = 4
            time.sleep(2)
            
                
        if (self.state_parking == 4):
            
            vel_msg.linear.x = 0.18
            vel_msg.angular.z = 0.0
            self.publisher.publish(vel_msg)
            
           
            if (laser[179]>self.d+0.15):
                vel_msg.linear.x = 0.0
                self.state_parking = 5
                self.publisher.publish(vel_msg)
                time.sleep(3)
        if (self.state_parking==5):
            vel_msg.linear.x = -0.14
            vel_msg.angular.z = 0.0
            self.publisher.publish(vel_msg)
            if (laser[179]<self.d+0.05):
                vel_msg.linear.x = 0.0
                self.publisher.publish(vel_msg)
                self.state_parking = 6
                
                time.sleep(3)
            

        if (self.state_parking==6):
            if self.park==1:
                vel_msg.angular.z = -1.05
                
            if self.park==2:
                vel_msg.angular.z = 1.34
            self.publisher.publish(vel_msg)

            if (laser[79]<self.d_r_l+0.4 or laser[281]<self.d_r_l+0.45):
                vel_msg.angular.z = 0.0
                self.state_parking = 7
                self.publisher.publish(vel_msg)
                time.sleep(3)
                self.state = 'none'
                 
        
    def lidar_callback(self, msg):
        if not self.is_started : return
        
        data = msg.ranges
        
        if self.state == 'avoid_blocks':
            self.avoid_blocks(data)
        elif self.state == 'pedestrian':
            left = np.array(data[:4])
            right = np.array(data[356:])
            center = np.concatenate((left, right), axis=None)
            zero = np.zeros_like(center)
            one = np.ones_like(center)
            mask = np.where(center==float('inf'), one, zero)
            res = np.all(mask)
            cmd_vel = Twist()
            if res:
                cmd_vel.linear.x = 0.4
                self.publisher.publish(cmd_vel)
            else: 
                cmd_vel.linear.x = 0.
                self.publisher.publish(cmd_vel)
            if data[90]<0.5:
                self.state == 'none'
     
    def traffic_light_callback(self, msg):
        cv_bridge = CvBridge()
        frame = cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if not self.is_started:
            if frame[300, 600][1]==109: self.is_started = 1

    def timer_callback(self):
        if not self.is_started: return
        
        match self.state:
            case 'none':
                new_vel = self.pid.update_error()
                self.publisher.publish(new_vel)


  
    def empty_listener_callback(self, msg):
        self.is_started = 1
        
    def recognizer_callback(self, msg):
        cv_bridge = CvBridge()
        frame = cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        #rec_signs = recognition(frame, *self.lab_data)
        #self.get_logger().info(f'{rec_signs}')
        
    def image_callback(self, msg):
        if not self.is_started: return
        
        cv_bridge = CvBridge()
        image = cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        self.pid.calc_error(image)
                 
        
def main(args=None):
    rclpy.init(args=args)
    starting = Starter()
    rclpy.spin(starting)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
