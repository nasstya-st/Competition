from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.node import Node
import rclpy
import sys
import math
import time

class Happy(Node):

     def __init__(self):
         # Creates a node with name 'turtlebot_controller' and make sure it is a
         # unique node (using anonymous=True).
         super().__init__('happy')

         # Publisher which will publish to the topic '/turtle1/cmd_vel'.
         self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 2)
         self.pose_subscriber = self.create_subscription(LaserScan, '/scan', self.update_pose, 10)
         self.scan = LaserScan()
         self.timer = self.create_timer(0.5, self.move)
         self.state_parking = 0
         self.park = 0
         self.d = 0
         self.d_r_l = 0
     def update_pose(self, ranges):
         self.scan = ranges

     def move(self):
         """Moves the turtle to the goal."""
         
         vel_msg = Twist()
         laser = self.scan.ranges
         if (len(laser)!=0):
            if(self.state_parking==0):
                if (laser[79]<0.41 or laser[281]<0.41):
                   
                    vel_msg.linear.x = 0.0
                    self.state_parking = 1
                else:

                    vel_msg.linear.x = 0.35
                    
                vel_msg.angular.z = 0.0
                self.velocity_publisher.publish(vel_msg)
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
                self.velocity_publisher.publish(vel_msg)
               
                if (laser[179]<0.25):
                    self.d = laser[179]
                    vel_msg.linear.x = 0.0
                    vel_msg.angular.z = 0.0
                    self.velocity_publisher.publish(vel_msg)
                    self.state_parking = 4
                time.sleep(2)
                
                    
            if (self.state_parking == 4):
                
                vel_msg.linear.x = 0.18
                vel_msg.angular.z = 0.0
                self.velocity_publisher.publish(vel_msg)
                
               
                if (laser[179]>self.d+0.15):
                    vel_msg.linear.x = 0.0
                    self.state_parking = 5
                    self.velocity_publisher.publish(vel_msg)
                    time.sleep(3)
            if (self.state_parking==5):
                vel_msg.linear.x = -0.14
                vel_msg.angular.z = 0.0
                self.velocity_publisher.publish(vel_msg)
                if (laser[179]<self.d+0.05):
                    vel_msg.linear.x = 0.0
                    self.velocity_publisher.publish(vel_msg)
                    self.state_parking = 6
                    
                    time.sleep(3)
                

            if (self.state_parking==6):
                if self.park==1:
                    vel_msg.angular.z = -0.9
                    
                if self.park==2:
                    vel_msg.angular.z = 1.34
                self.velocity_publisher.publish(vel_msg)

                if (laser[79]<self.d_r_l+0.4 or laser[281]<self.d_r_l):
                    vel_msg.angular.z = 0.0
                    self.state_parking = 7
                    self.velocity_publisher.publish(vel_msg)
                    time.sleep(3)
                
                    

            if (self.state_parking == 7):
                 k = 0
                 l = 0
                 if (self.park == 1 and  l==0):
                    vel_msg.linear.x = 0.2
                    self.velocity_publisher.publish(vel_msg)
                    time.sleep(3)
                    l = 1   

                 vel_msg.linear.x = 0.2
                 self.velocity_publisher.publish(vel_msg)
                 for i in range (30):
                    if laser[300+i]<0.3:
                        k=1
                 if (k == 1):
                    vel_msg.linear.x  = 0.0
                    self.velocity_publisher.publish(vel_msg) 
                    self.state_parking = 8
                    time.sleep(2)
            if (self.state_parking == 8):
                vel_msg.angular.z = 0.8
                k = 0
                self.velocity_publisher.publish(vel_msg)
                
                for i in range (35):
                    if laser[185+i]<0.3:
                        k=1
                
                
                if (k == 1):
                    vel_msg.angular.z  = 0.0
                    self.velocity_publisher.publish(vel_msg) 
                    quit()   




         
 
         
def main(args=None):
    rclpy.init(args=args)
    x = Happy()
    rclpy.spin(x)
    x.destroy_node()
    rclpy.shutdown()

 
if __name__ == '__main__':
    main()
