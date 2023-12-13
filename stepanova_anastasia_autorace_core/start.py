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

#from stepanova_anastasia_autorace_core.action import AvoidBlocks

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
        Kp, Ki, Kd = (0.0185, 0.0, 0.019)
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
        #self.avoid_blocks = ActionClient(self, AvoidBlocks, 'execute_turtle_commands')
         	    
        self.timer_period = 0.2
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.pid = PID(self.timer_period)
        
        self.lab_data = lab()
        #self.get_logger.info(f'{self.lab_data[-1]}')
        
        self.is_started = 0
        self.state = 'avoid_blocks'
        #self.lidar_data = 0
        self.avoid_blocks_state = 1
        
    def change_avoid_blocks_state(self, state):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.
        cmd_vel.angular.z = 0.
        self.publisher.publish(cmd_vel)
        self.avoid_blocks_state = state
        
    def lidar_callback(self, msg):
        if not self.is_started or self.state != 'avoid_blocks': return
        data = msg.ranges
        cmd_vel = Twist()
        '''
        self.get_logger().info(f'90 {data[90]}')
        time.sleep(1)
        self.get_logger().info(f'0 {data[0]}')
        time.sleep(1)'''
        self.get_logger().info(f'{self.avoid_blocks_state}')
        #self.get_logger().info(f'0 {data[0]}')
        
        match self.avoid_blocks_state:
            case 1:
                if data[90] < 0.4:
                    cmd_vel.linear.x = 0.3
                    cmd_vel.angular.z = 0.
                    self.publisher.publish(cmd_vel)
                else:
                    self.change_avoid_blocks_state(2)
            case 2:
                if data[0] > 0.8:
                    cmd_vel.linear.x = 0.
                    cmd_vel.angular.z = 0.5
                    self.publisher.publish(cmd_vel)
                else:
                    self.change_avoid_blocks_state(3)
            case 3:
                if data[0] > 0.25:
                    cmd_vel.linear.x = 0.15
                    cmd_vel.angular.z = 0.
                    self.publisher.publish(cmd_vel)
                else:
                    self.change_avoid_blocks_state(4)
            case 4:
                if data[270] > 0.25:
                    cmd_vel.linear.x = 0.
                    cmd_vel.angular.z = 0.4
                    self.publisher.publish(cmd_vel)
                else:
                    self.change_avoid_blocks_state(5)    
            case 5:
                if data[270] < 0.25:
                    cmd_vel.linear.x = 0.2
                    cmd_vel.angular.z = 0.
                    self.publisher.publish(cmd_vel)
                else:
                    self.change_avoid_blocks_state(6)     
            case 6:
                if data[0] > 0.66:
                    cmd_vel.linear.x = 0.
                    cmd_vel.angular.z = -0.4
                    self.publisher.publish(cmd_vel)
                else:
                    self.change_avoid_blocks_state(7)      
            case 7:
                if data[0] > 0.3:
                    cmd_vel.linear.x = 0.2
                    cmd_vel.angular.z = 0.
                    self.publisher.publish(cmd_vel)
                else:
                    self.change_avoid_blocks_state(8)                      
            case 8:
                if data[90] > 0.3:
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
                if data[180] > 0.15:
                    cmd_vel.linear.x = 0.
                    cmd_vel.angular.z = 0.3
                    self.publisher.publish(cmd_vel)
                else:
                    self.change_avoid_blocks_state(-1)                  
                    self.state = 'none'        
                
                                                                           
        '''
        if data[130] > 1 or data[130] == float('inf'):
            cmd_vel.linear.x = 0.2
            self.publisher.publish(cmd_vel)
        else: self.lidar_state=1
        
        if self.lidar_state==1:
            cmd_vel.linear.x = 0.
            cmd_vel.angular.z = 3.14/2
            self.publisher.publish(cmd_vel)
            time.sleep(1)
            cmd_vel.linear.x = 0.1
            cmd_vel.angular.z = 0.
            time.sleep(1)
            self.publisher.publish(cmd_vel)
            cmd_vel.linear.x = 0.
            cmd_vel.angular.z = 3.14/2
            self.publisher.publish(cmd_vel)
            time.sleep(1)
            cmd_vel.linear.x = 0.
            cmd_vel.angular.z = -3.14/2
            self.publisher.publish(cmd_vel)
            time.sleep(1)
            self.lidar_state=2'''
        
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
            case 'avoid_blocks':
                pass
                '''
                data = self.lidar_data
                n = len(data)
                self.avoid_blocks()
                #self.get_logger().info(f'{data[95]}')
                
                if data[92] > 1:
                    new_vel = self.pid.update_error()
                    self.publisher.publish(new_vel)
                    #self.get_logger().info(f'{data[95]}')
                else: self.avoid_blocks()'''
  
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
        #self.get_logger().info(f'calculating error')
        #rec_signs = recognition(gray, *self.lab_data)
        #self.get_logger().info(f'{rec_signs}')
        
        
    def avoid_blocks(self):
        data = self.lidar_data
        n = len(data)
        #print(data[n//2])
        while 3:
            self.get_logger().info(f'{data[95]}')
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.2
            self.publisher.publish(cmd_vel)

        '''
        while data[n//2] > 2:
            #self.get_logger().info(f'{data[n//2]}')
            new_vel = self.pid.update_error()
            self.publisher.publish(new_vel)
        self.get_logger().info(f'here0')
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.2
        cmd_vel.angular.z = 6.14
        self.publisher.publish(cmd_vel)
        time.sleep(4)
        self.get_logger().info(f'here1')
        cmd_vel.linear.x = 0.2
        cmd_vel.angular.z = -6.14
        self.publisher.publish(cmd_vel)
        time.sleep(5)
        self.get_logger().info(f'here2')
        cmd_vel.linear.x = 0.2
        cmd_vel.angular.z = 6.14
        self.publisher.publish(cmd_vel)
        time.sleep(6)
        self.get_logger().info(f'here3')'''

        self.state = 'none'    
        
def main(args=None):
    rclpy.init(args=args)

    starting = Starter()

    rclpy.spin(starting)

    #circling.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
