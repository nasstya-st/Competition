import rclpy, sys
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class CirclePublisher(Node):

    def __init__(self):
        super().__init__('publisher')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Empty, "robot_start",
         	    self.listener_callback, 10)
        self.subscription = self.create_subscription(Image, "/color/image_projected_compensated",
         	    self.image_callback, 5) 	    
        self.timer_period = 0.5 
        self.error = [0,0]
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.curr_time = 0
        self.prevpt1 = [100,100]
        self.prevpt2 = [700,100]

    def timer_callback(self):
        self.curr_time += self.timer_period
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.1
        cmd_vel.angular.z = float( \
        			        0.005 * self.error[-1] + \
                            0.00 * np.sum(np.array(self.error)*self.timer_period) + \
                            0.003 * (self.error[-1] - self.error[-2]) / self.timer_period )
        self.get_logger().info(f'{cmd_vel.angular.z} z')
        self.publisher.publish(cmd_vel)

  
    def listener_callback(self, msg):
        twist = Twist()
        twist.linear.x = 0.5      
        self.publisher.publish(twist)
        
    def get_line_markings(self, frame):
        hls = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)
        _, sxbinary = edge.threshold(hls[:, :, 1], thresh=(120, 255))
        sxbinary = edge.blur_gaussian(sxbinary, ksize=3) # Reduce noise
        sxbinary = edge.mag_thresh(sxbinary, sobel_kernel=3, thresh=(110, 255))
        _, s_binary = edge.threshold(s_channel, (80, 255))
        _, r_thresh = edge.threshold(frame[:, :, 2], thresh=(120, 255))
        rs_binary = cv2.bitwise_and(s_binary, r_thresh)
        lane_line_markings = cv2.bitwise_or(rs_binary, sxbinary.astype(np.uint8))    
        return lane_line_markings
        
    def histogram_peak(self, histogram):
        """
        Get the left and right peak of the histogram
     
        Return the x coordinate of the left histogram peak and the right histogram
        peak.
        """
        midpoint = np.int(histogram.shape[0]/2)
        leftx_base = np.argmax(histogram[:midpoint])
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint
     
        return leftx_base, rightx_base
        
        
    def get_lane_line_indices_sliding_windows(self, warped_frame, histogram):
        """
        Get the indices of the lane line pixels using the 
        sliding windows technique.
             
        :return: Best fit lines for the left and right lines of the current lane 
        """
        # Sliding window width is +/- margin
        margin = 71
        no_of_windows = 10
     
        frame_sliding_window = warped_frame.copy()
     
        # Set the height of the sliding windows
        window_height = np.int(warped_frame.shape[0]/no_of_windows)       
     
        # Find the x and y coordinates of all the nonzero 
        # (i.e. white) pixels in the frame. 
        nonzero = warped_frame.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1]) 
             
        # Store the pixel indices for the left and right lane lines
        left_lane_inds = []
        right_lane_inds = []
             
        # Current positions for pixel indices for each window,
        # which we will continue to update
        leftx_current, rightx_current = self.histogram_peak(histogram)
     
        # Go through one window at a time
             
        for window in range(no_of_windows):
           
          # Identify window boundaries in x and y (and right and left)
          win_y_low = warped_frame.shape[0] - (window + 1) * window_height
          win_y_high = warped_frame.shape[0] - window * window_height
          win_xleft_low = leftx_current - margin
          win_xleft_high = leftx_current + margin
          win_xright_low = rightx_current - margin
          win_xright_high = rightx_current + margin
          #cv2.rectangle(frame_sliding_window,(win_xleft_low,win_y_low),(
          #win_xleft_high,win_y_high), (255,255,255), 2)
          #cv2.rectangle(frame_sliding_window,(win_xright_low,win_y_low),(
          #win_xright_high,win_y_high), (255,255,255), 2)
     
          # Identify the nonzero pixels in x and y within the window
          good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
                              (nonzerox >= win_xleft_low) & (
                               nonzerox < win_xleft_high)).nonzero()[0]
          good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
                               (nonzerox >= win_xright_low) & (
                                nonzerox < win_xright_high)).nonzero()[0]
                                                             
          # Append these indices to the lists
          left_lane_inds.append(good_left_inds)
          right_lane_inds.append(good_right_inds)
             
          # If you found > minpix pixels, recenter next window on mean position
          minpix = int((1/24) * 848)
          if len(good_left_inds) > minpix:
            leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
          if len(good_right_inds) > minpix:        
            rightx_current = np.int(np.mean(nonzerox[good_right_inds]))
                         
        # Concatenate the arrays of indices
        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)
     
        # Extract the pixel coordinates for the left and right lane lines
        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds] 
        rightx = nonzerox[right_lane_inds] 
        righty = nonzeroy[right_lane_inds]
     
        # Fit a second order polynomial curve to the pixel coordinates for
        # the left and right lane lines
        left_fit = np.polyfit(lefty, leftx, 2)
        right_fit = np.polyfit(righty, rightx, 2)  
        
            
         
        # Create the x and y values to plot on the image  
        ploty = np.linspace(
          0, frame_sliding_window.shape[0]-1, frame_sliding_window.shape[0])
        left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
        right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]
   
        # Generate an image to visualize the result
        out_img = np.dstack((
          frame_sliding_window, frame_sliding_window, (
          frame_sliding_window))) * 255
               
        # Add color to the left line pixels and right line pixels
        out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
        out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [
          0, 0, 255]
                   
        # Plot the figure with the sliding windows
        figure, (ax1, ax2, ax3) = plt.subplots(3,1) # 3 rows, 1 column
        figure.set_size_inches(10, 10)
        figure.tight_layout(pad=3.0)
        ax1.imshow(cv2.cvtColor(self.orig_frame, cv2.COLOR_BGR2RGB))
        ax2.imshow(frame_sliding_window, cmap='gray')
        ax3.imshow(out_img)
        ax3.plot(left_fitx, ploty, color='yellow')
        ax3.plot(right_fitx, ploty, color='yellow')
        ax1.set_title("Original Frame")  
        ax2.set_title("Warped Frame with Sliding Windows")
        ax3.set_title("Detected Lane Lines with Sliding Windows")
        plt.show()
                 
        return left_fit, right_fit
        
    def get_lane_line_previous_window(self, warped_frame, left_fit, right_fit):
        """
        Use the lane line from the previous sliding window to get the parameters
        for the polynomial line for filling in the lane line
        :param: left_fit Polynomial function of the left lane line
        :param: right_fit Polynomial function of the right lane line
        """
        # margin is a sliding window parameter
        margin = 71
     
        # Find the x and y coordinates of all the nonzero 
        # (i.e. white) pixels in the frame.         
        nonzero = warped_frame.nonzero()  
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
             
        # Store left and right lane pixel indices
        left_lane_inds = ((nonzerox > (left_fit[0]*(
          nonzeroy**2) + left_fit[1]*nonzeroy + left_fit[2] - margin)) & (
          nonzerox < (left_fit[0]*(
          nonzeroy**2) + left_fit[1]*nonzeroy + left_fit[2] + margin))) 
        right_lane_inds = ((nonzerox > (right_fit[0]*(
          nonzeroy**2) + right_fit[1]*nonzeroy + right_fit[2] - margin)) & (
          nonzerox < (right_fit[0]*(
          nonzeroy**2) + right_fit[1]*nonzeroy + right_fit[2] + margin)))           

     
        # Get the left and right lane line pixel locations  
        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds] 
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]  
         
        # Fit a second order polynomial curve to each lane line
        left_fit = np.polyfit(lefty, leftx, 2)
        right_fit = np.polyfit(righty, rightx, 2)

        # Create the x and y values to plot on the image
        ploty = np.linspace(
          0, self.warped_frame.shape[0]-1, self.warped_frame.shape[0]) 
        left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
        right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]    
        
        return leftx, rightx, lefty, righty, left_fit, right_fit, left_lane_inds, right_lane_inds, ploty, left_fitx, right_fitx
        
    def overlay_lane_lines(self, warped_frame, ploty, left_fitx, right_fitx):
        """
        Overlay lane lines on the original frame

        :return: Lane with overlay
        """
        # Generate an image to draw the lane lines on 
        warp_zero = np.zeros_like(warped_frame).astype(np.uint8)
        color_warp = np.dstack((warp_zero, warp_zero, warp_zero))       
             
        # Recast the x and y points into usable format for cv2.fillPoly()
        pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
        pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
        pts = np.hstack((pts_left, pts_right))
             
        # Draw lane on the warped blank image
        cv2.fillPoly(color_warp, np.int_([pts]), (0,255, 0))
     
        # Warp the blank back to original image space using inverse perspective 
        # matrix (Minv)
        newwarp = cv2.warpPerspective(color_warp, self.inv_transformation_matrix, (
                                      self.orig_frame.shape[
                                      1], self.orig_frame.shape[0]))
         
        # Combine the result with the original image
        result = cv2.addWeighted(self.orig_frame, 1, newwarp, 0.3, 0) 
     
        return result
        
    def calculate_curvature(self, leftx, rightx, ploty, lefty, righty):
        """
        Calculate the road curvature in meters.
     
        :param: print_to_terminal Display data to console if True
        :return: Radii of curvature
        """
        # Set the y-value where we want to calculate the road curvature.
        # Select the maximum y-value, which is the bottom of the frame.
        y_eval = np.max(ploty)  
        
        # Pixel parameters for x and y dimensions
        YM_PER_PIX = 10.0 / 640 # meters per pixel in y dimension  !!!!!!!
        XM_PER_PIX = 3.7 / 848 # meters per pixel in x dimension   !!!!!!!
           
     
        # Fit polynomial curves to the real world environment
        left_fit_cr = np.polyfit(lefty * YM_PER_PIX, leftx * (
          XM_PER_PIX), 2)
        right_fit_cr = np.polyfit(righty * YM_PER_PIX, rightx * (
          XM_PER_PIX), 2)
                 
        # Calculate the radii of curvature
        left_curvem = ((1 + (2*left_fit_cr[0]*y_eval*YM_PER_PIX + left_fit_cr[
                        1])**2)**1.5) / np.absolute(2*left_fit_cr[0])
        right_curvem = ((1 + (2*right_fit_cr[
                        0]*y_eval*YM_PER_PIX + right_fit_cr[
                        1])**2)**1.5) / np.absolute(2*right_fit_cr[0])
   
     
        return left_curvem, right_curvem
        
    def calculate_car_position(self, orig_frame, left_fit, right_fit):
        """
        Calculate the position of the car relative to the center
             
        :param: print_to_terminal Display data to console if True       
        :return: Offset from the center of the lane
        """
        # Assume the camera is centered in the image.
        # Get position of car in centimeters
        car_location = orig_frame.shape[1] / 2
     
        # Fine the x coordinate of the lane line bottom
        height = orig_frame.shape[0]
        bottom_left = left_fit[0]*height**2 + left_fit[1]*height + left_fit[2]
        bottom_right = right_fit[0]*height**2 + right_fit[1]*height + right_fit[2]
     
        YM_PER_PIX = 10.0 / 640 # meters per pixel in y dimension  !!!!!!!
        XM_PER_PIX = 3.7 / 848 # meters per pixel in x dimension   !!!!!!!     
     
        center_lane = (bottom_right - bottom_left)/2 + bottom_left 
        center_offset = (np.abs(car_location) - np.abs(
          center_lane)) * XM_PER_PIX * 100
           
        return center_offset
    
    def image_callback(self, msg):
        cv_bridge = CvBridge()
        frame = cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        #lane_obj = Lane(orig_frame=frame)
        #lane_line_markings = lane_obj.get_line_markings()
        #warped_frame = lane_obj.perspective_transform(plot=False)
        histogram = np.sum(frame[int(frame.shape[0]/2):,:], axis=0)
        left_fit, right_fit = self.get_lane_line_indices_sliding_windows(frame, histogram)
        leftx, rightx, lefty, righty, left_fit, right_fit, left_lane_inds, right_lane_inds, ploty, left_fitx, right_fitx = self.get_lane_line_previous_window(left_fit, right_fit)
        
        frame_with_lane_lines = self.overlay_lane_lines(frame, ploty, left_fitx, right_fitx)
        left_curvem, right_curvem = self.calculate_curvature(leftx, rightx, ploty, lefty, righty)          
                         
        error = self.calculate_car_position(frame, left_fit, right_fit)
        self.get_logger().info(f'{error} error')
        self.error.append()
        
        
        
def main(args=None):
    rclpy.init(args=args)

    circling = CirclePublisher()

    rclpy.spin(circling)

    #circling.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
