import rclpy, sys, time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, String
from sensor_msgs.msg import Image, LaserScan

from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import matplotlib.pyplot as plt
from .pid import PID


class Starter(Node):

    def __init__(self):
        super().__init__('publisher')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 1)
        self.finish_publisher = self.create_publisher(String, 'robot_finish', 1)
        self.subscription = self.create_subscription(Empty, "robot_start", self.empty_listener_callback, 5)
        self.sign_reader = self.create_subscription(String, "signes", self.sign_reader_callback, 1)
        self.subscription = self.create_subscription(Image, "/color/image_projected_compensated",
         	    self.image_callback, 1) 	    
        self.starter = self.create_subscription(Image, "/color/image",
         	    self.traffic_light_callback, 1) 
        #self.recognizer = self.create_subscription(Image, "/color/image", self.recognizer_callback, 1) 
        self.lidar = self.create_subscription(LaserScan, "/scan", self.lidar_callback, 1)  	    
        self.publisher1 = self.create_publisher(String, "robot_stop", 1)
         	    
        self.timer_period = 0.2
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.pid = PID(self.timer_period)
        
        self.is_started = 0
        self.is_reading = 1
        #states = ['none', 'pedestrian_crossing_sign', 'traffic_construction', 
        #          'traffic_parking', 'traffic_intersection', 'tunnel', 'finish']
        self.state = 'none'
        self.avoid_blocks_state = 0  # 0 before mission
        
        self.state_turn = -1
        self.l_r = "" # left или right, зависит от определения знака

        self.state_parking = 0
        self.park = 0  # left or right
        self.d_r_l = 0 # distance to parked car
        self.flag1 = 0
        
    def sign_reader_callback(self, msg):
        '''
    	gets information about detected signes
    	and changes state correspondingly
        '''
        if not self.is_reading: return
        state = msg.data
        if state == 'traffic_left': self.l_r = 'left'
        elif state == 'traffic_right': self.l_r = 'right'
        #elif state == 'traffic_intersection': self.state = 'intersection'
        self.state = state
        if self.state!='none' and self.state!='traffic_intersection': self.is_reading =  0
        self.get_logger().info(f'{self.state}')
        
    def change_avoid_blocks_state(self, state):
        '''
        subfunction to change states
        '''
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.
        cmd_vel.angular.z = 0.
        self.publisher.publish(cmd_vel)
        self.avoid_blocks_state = state
        
        
    def avoid_blocks(self, data):
        '''
        function for construction mission
        '''
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
                    
    def parking(self, laser):
        '''
        function for parking mission
        '''        
        vel_msg = Twist()
        if (len(laser)==0): return

        if self.state_parking==0:
            tmp = 0
            for i in range (20):
                if laser[250+i]<0.3:
                    tmp=1
                    
            if tmp:
                vel_msg.linear.x = 0.0
                self.state_parking = 1
            else:
                vel_msg.linear.x = 0.25
                
            vel_msg.angular.z = 0.0
            self.publisher.publish(vel_msg)
            
        if self.state_parking == 1:
            tmp = 0
            for i in range (10):
                if laser[130+i]<0.3:
                    tmp=1
                    
            if tmp:
                vel_msg.angular.z = 0.0
                self.state_parking = 2
            else:
                vel_msg.angular.z = 0.58
        
            self.publisher.publish(vel_msg)
            
        if(self.state_parking==2):
            if (laser[79]<0.41 or laser[281]<0.41):
                vel_msg.linear.x = 0.0
                self.state_parking = 3
            else:
                vel_msg.linear.x = 0.35
                
            vel_msg.angular.z = 0.0
            self.publisher.publish(vel_msg)
            if (self.state_parking==3):
                if (laser[79]<0.41):
                    self.d_r_l = laser[79]
                    self.state_parking = 4
                    self.park = 1
                if (laser[281]<0.41):
                    self.d_r_l = laser[281]
                    self.state_parking = 5
                    self.park = 2
               
                time.sleep(2)
                
        if (self.state_parking==4 or self.state_parking ==5):
            if self.state_parking==4:
                vel_msg.angular.z = -0.4
            if self.state_parking==5:
                vel_msg.angular.z = 0.4
            self.publisher.publish(vel_msg)
           
            if (laser[179]<0.2):
                self.d = laser[179]
                vel_msg.linear.x = 0.0
                vel_msg.angular.z = 0.0
                self.publisher.publish(vel_msg)
                self.state_parking = 6
                time.sleep(2)
            
                
        if (self.state_parking == 6):
            
            vel_msg.linear.x = 0.18
            vel_msg.angular.z = 0.0
            self.publisher.publish(vel_msg)
            
           
            if self.park == 2:
                if (laser[179]>self.d+0.15):
                    vel_msg.linear.x = 0.0
                    self.state_parking = 7
                    self.publisher.publish(vel_msg)
                    time.sleep(3)
            else:
                if (laser[179]>self.d+0.2):
                    vel_msg.linear.x = 0.0
                    self.state_parking = 7
                    self.publisher.publish(vel_msg)
                    time.sleep(3)
        if (self.state_parking==7):
            vel_msg.linear.x = -0.14
            vel_msg.angular.z = 0.0
            self.publisher.publish(vel_msg)
            if (laser[179]<self.d+0.05):
                vel_msg.linear.x = 0.0
                self.publisher.publish(vel_msg)
                self.state_parking = 8
                
                time.sleep(3)
            

        if (self.state_parking==8):
            if self.park==1:
                vel_msg.angular.z = -1.05
                
            if self.park==2:
                vel_msg.angular.z = 1.34
            self.publisher.publish(vel_msg)

            if (laser[79]<self.d_r_l+0.4 or laser[281]<self.d_r_l+0.45):
                vel_msg.angular.z = 0.0
                self.state_parking = 9
                self.publisher.publish(vel_msg)
                time.sleep(3)
                self.state = 'none'
                 
                 
    def intersection(self, laser):
        '''
        function for intersection mission
        '''
        vel_msg=Twist()
        if (len(laser)==0): return
         
        if self.flag1==0:
            for i in range (20):
                if laser[329+i]<1.0:
                    self.flag1 = 1
                    self.l_r = "right"
                if self.flag1==0:
                    self.flag1 = 1
                    self.l_r = "left"
        #self.get_logger().info(f'{self.flag1}')
        if self.state_turn == -1:
            vel_msg.linear.x = 0.06
            vel_msg.angular.z = 0.
            tmp = 0
            for i in range (20):
                if laser[20+i]<0.2 or laser[310+i]<0.2:
                    tmp=1
            if tmp:
                vel_msg.linear.x = 0.
                self.state_turn = 0
            self.publisher.publish(vel_msg)
        if (self.l_r == 'right'):
            if(self.state_turn == 0):
                vel_msg.linear.x = 0.
                vel_msg.angular.z = -0.4
                tmp = 0
                for i in range (10):
                    if laser[60+i]<0.2:
                        tmp=1
                if tmp:
                    vel_msg.angular.z = 0.
                    self.state_turn = 1
                self.publisher.publish(vel_msg)
            if (self.state_turn == 1):
                vel_msg.linear.x = 0.32
                vel_msg.angular.z = 1.
                tmp = 0
                for i in range (20):
                    if laser[140+i]<0.7 and laser[140+i]>0.25:
                        tmp=1
                if tmp:
                    vel_msg.linear.x = 0.
                    vel_msg.angular.z = 0.
                    self.state_turn = 2
                self.publisher.publish(vel_msg)
            if (self.state_turn == 2):
                vel_msg.angular.z = -0.4
                tmp = 0
                for i in range (20):
                    if laser[190+i]<0.5:
                        tmp=1
                if tmp:
                    vel_msg.angular.z = 0.
                    self.state_turn = 3
                self.publisher.publish(vel_msg)
            if (self.state_turn == 3):
                self.pid.velocity = 0.1
                self.state = 'finish'
                self.is_reading =  1

        elif (self.l_r == 'left'):
            if(self.state_turn == 0):
                vel_msg.linear.x = 0.
                vel_msg.angular.z = 0.4
                tmp = 0
                for i in range (10):
                    if laser[290+i]<0.3 :
                        tmp=1 
                if tmp:
                    vel_msg.angular.z = 0.
                    self.state_turn = 1
                self.publisher.publish(vel_msg)
            if (self.state_turn == 1):
                vel_msg.linear.x = 0.32
                vel_msg.angular.z = -1.
                tmp = 0
                for i in range (20):
                    if laser[270+i]<0.7 and laser[270+i]>0.3:
                        tmp=1
                if tmp:
                    vel_msg.linear.x = 0.
                    vel_msg.angular.z = 0.
                    self.state_turn = 2
                self.publisher.publish(vel_msg)
            if (self.state_turn == 2):
                vel_msg.angular.z = 0.4
                tmp = 0
                for i in range (20):
                    if laser[175+i]<0.5:
                        tmp=1
                if tmp:
                    vel_msg.linear.x = 0.0
                    vel_msg.angular.z = 0.
                    self.state_turn = 3
                self.publisher.publish(vel_msg)
            if (self.state_turn == 3):
                self.state = 'finish'
                self.pid.velocity = 0.
                self.is_reading =  1
        
    def lidar_callback(self, msg):
        '''
        gets lidar information and 
        calls functions if needed
        '''
        if not self.is_started : return
        
        data = msg.ranges
        
        if self.state == 'traffic_construction':
            self.avoid_blocks(data)
        elif self.l_r:
            self.intersection(data)
        elif self.state == 'traffic_parking':
            self.parking(data)
        elif self.state == 'pedestrian_crossing_sign':
            left = np.array(data[:4])
            right = np.array(data[356:])
            center = np.concatenate((left, right), axis=None)
            zero = np.zeros_like(center)
            one = np.ones_like(center)
            mask = np.where(center>0.5, one, zero)
            res = np.all(mask)
            cmd_vel = Twist()
            if res:
                cmd_vel.linear.x = 0.2
                self.publisher.publish(cmd_vel)
            else: 
                cmd_vel.linear.x = 0.
                self.publisher.publish(cmd_vel)
            if data[90]<0.5:
                cmd_vel.linear.x = 0.
                self.publisher.publish(cmd_vel)
                self.state = 'finish'
            #self.get_logger().info(f'{res, center, data[90]}')
     
    def traffic_light_callback(self, msg):
        '''
        checks the traffic light and starts robot
        '''
        cv_bridge = CvBridge()
        frame = cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if not self.is_started:
            if frame[300, 600][1]==109: self.is_started = 1

    def timer_callback(self):
        '''
        calls speed update function
        or finishes ride
        '''
        if not self.is_started: return
        
        match self.state:
            case 'none':
                new_vel = self.pid.update_error()
                self.publisher.publish(new_vel)
                #self.get_logger().info(f'in pid')
            case 'finish':
                msg = String()
                msg.data = 'fredy fazbear ur ur ur ur'
                self.finish_publisher.publish(msg)
                cmd_vel = Twist()
                cmd_vel.linear.x = 0.
                self.publisher.publish(cmd_vel)
        msg = String()
        if self.state!='none': msg.data = 'stop'
        else: msg.data = 'start'
        
        self.publisher1.publish(msg)
            
    def empty_listener_callback(self, msg):
        '''
        starts robot from topic robot_start
        '''
        self.is_started = 1
        
    '''def recognizer_callback(self, msg):
        cv_bridge = CvBridge()
        frame = cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        #rec_signs = recognition(frame, *self.lab_data)
        #self.get_logger().info(f'{rec_signs}')'''
        
    def image_callback(self, msg):
        '''
        gets image for pid to calculate error
        '''
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
    
