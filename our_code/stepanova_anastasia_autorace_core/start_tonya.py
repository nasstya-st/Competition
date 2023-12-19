import rclpy, sys
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from sensor_msgs.msg import Image
import matplotlib.pyplot as plt
from cv_bridge import CvBridge
import cv2
import numpy as np

class CirclePublisher(Node):

    def __init__(self):
        super().__init__('publisher')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 1)
        self.subscription = self.create_subscription(Empty, "robot_start",
               self.listener_callback, 10)
        self.subscription = self.create_subscription(Image, "/color/image_projected_compensated",
               self.image_callback, 1)       
        self.timer_period = 0.5 
        self.error = [0,0]
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.prevpt1 = [100,100]
        self.prevpt2 = [700,100]
        self.curr_time = 0

    def timer_callback(self):
        self.curr_time += self.timer_period
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.1
        cmd_vel.angular.z = float( \
                      0.018* self.error[-1] + \
                      0.0000 * np.sum(np.array(self.error)*self.timer_period) + \
                      0.017* (self.error[-1] - self.error[-2]) / self.timer_period )
        #self.get_logger().info(f'{cmd_vel.angular.z}')
        self.publisher.publish(cmd_vel)

  
    def listener_callback(self, msg):
        twist = Twist()
        twist.linear.x = 0.05    
        self.publisher.publish(twist)
        
    def image_callback(self, msg):
        cv_bridge = CvBridge()
        gray = cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        b1 = 232
        g1 = 0
        r1 = 0
        b2 = 255
        g2 = 255
        r2 = 255
        h_min = (g1, b1, r1)
        h_max = (g2, b2, r2)

        gray = cv2.inRange(gray, h_min, h_max)
        
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

        
        
def main(args=None):
    rclpy.init(args=args)

    circling = CirclePublisher()

    rclpy.spin(circling)

    #circling.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
