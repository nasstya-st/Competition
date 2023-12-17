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
         self.state_turn = 0
         self.l_r = "right" #или right, зависит от определения знака
          
     def update_pose(self, ranges):
         self.scan = ranges

     def move(self):
         """Moves the turtle to the goal."""
         
         vel_msg = Twist()
         laser = self.scan.ranges
         self.get_logger().info(f'{self.state_turn}')
         if (len(laser)!=0):
            if (self.l_r == 'left'):
                if(self.state_turn == 0):
                    vel_msg.linear.x = 0.
                    vel_msg.angular.z = -0.4
                    tmp = 0
                    for i in range (20):
                        if laser[110+i]<0.3:
                            tmp=1
                    if tmp:
                        vel_msg.angular.z = 0.
                        self.state_turn = 1
                    self.velocity_publisher.publish(vel_msg)
                if (self.state_turn == 1):
                    vel_msg.linear.x = 0.3
                    vel_msg.angular.z = 1.
                    tmp = 0
                    for i in range (20):
                        if laser[110+i]<0.7 and laser[110+i]>0.3:
                            tmp=1
                    if tmp:
                        vel_msg.linear.x = 0.
                        vel_msg.angular.z = 0.
                        self.state_turn = 2
                    self.velocity_publisher.publish(vel_msg)
                if (self.state_turn == 2):
                    vel_msg.linear.x = 0.2
                    vel_msg.angular.z = 0.8
                    tmp = 0
                    for i in range (20):
                        if laser[30+i]<0.9 and laser[30+i]>0.3:
                            tmp=1
                    if tmp:
                        vel_msg.linear.x = 0.
                        vel_msg.angular.z = 0.
                        self.state_turn = 3
                    self.velocity_publisher.publish(vel_msg)
                if (self.state_turn == 3):
                    vel_msg.angular.z = -0.4
                    tmp = 0
                    for i in range (20):
                        if laser[160+i]<0.5:
                            tmp=1
                    if tmp:
                        vel_msg.angular.z = 0.
                        self.state_turn = 4
                    self.velocity_publisher.publish(vel_msg)
                if (self.state_turn == 4):
                    quit() 

            elif (self.l_r == 'right'):
                if(self.state_turn == 0):
                    vel_msg.linear.x = 0.
                    vel_msg.angular.z = 0.4
                    tmp = 0
                    for i in range (20):
                        if laser[250+i]<0.3:
                            tmp=1 
                    if tmp:
                        vel_msg.angular.z = 0.
                        self.state_turn = 1
                    self.velocity_publisher.publish(vel_msg)
                if (self.state_turn == 1):
                    vel_msg.linear.x = 0.3
                    vel_msg.angular.z = -1.
                    tmp = 0
                    for i in range (20):
                        if laser[250+i]<0.7 and laser[250+i]>0.3:
                            tmp=1
                    if tmp:
                        vel_msg.linear.x = 0.
                        vel_msg.angular.z = 0.
                        self.state_turn = 2
                    self.velocity_publisher.publish(vel_msg)
                if (self.state_turn == 2):
                    vel_msg.linear.x = 0.2
                    vel_msg.angular.z = -0.8
                    tmp = 0
                    for i in range (20):
                        if laser[330+i]<0.9 and laser[330+i]>0.3:
                            tmp=1
                    if tmp:
                        vel_msg.linear.x = 0.
                        vel_msg.angular.z = 0.
                        self.state_turn = 3
                    self.velocity_publisher.publish(vel_msg)
                if (self.state_turn == 3):
                    vel_msg.angular.z = 0.4
                    tmp = 0
                    for i in range (20):
                        if laser[200+i]<0.5:
                            tmp=1
                    if tmp:
                        vel_msg.angular.z = 0.
                        self.state_turn = 4
                    self.velocity_publisher.publish(vel_msg)
                if (self.state_turn == 4):
                    quit()  
        
                
                
                    

            




         
 
         
def main(args=None):
    rclpy.init(args=args)
    x = Happy()
    rclpy.spin(x)
    x.destroy_node()
    rclpy.shutdown()

 
if __name__ == '__main__':
    main()
