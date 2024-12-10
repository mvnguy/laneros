#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge

# Initialize global variables
# Converts ROS image messages to OpenCV images
bridge = CvBridge()
# Height and width of the expected input image
PICTURE_HEIGHT = 480
PICTURE_WIDTH = 640
# Center position of the robot in the image
TRUCK_POS_X = PICTURE_WIDTH // 2  

def calculate_lane_center(image):
    """
    Detect the left and right lane markings and calculate the lane center.
    :param image: Input image in OpenCV format
    :return: Offset from the center of the robot to the lane center
    """
    # Step 1: Convert the image to grayscale for simpler processing
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Step 2: Apply Gaussian blur to smooth the image and reduce noise
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # Step 3: Use Canny edge detection to identify strong edges in the image
    edges = cv2.Canny(blurred, 50, 150)
    
    # Step 4: Define a region of interest (ROI) to focus on the road area
    mask = np.zeros_like(edges)
    roi_corners = np.array([[(50, PICTURE_HEIGHT), (PICTURE_WIDTH - 50, PICTURE_HEIGHT), 
                             (PICTURE_WIDTH // 2 + 50, PICTURE_HEIGHT // 2), 
                             (PICTURE_WIDTH // 2 - 50, PICTURE_HEIGHT // 2)]], dtype=np.int32)
    cv2.fillPoly(mask, roi_corners, 255)
    cropped_edges = cv2.bitwise_and(edges, mask)
    
    # Step 5: Detect lines in the ROI using Hough Line Transform
    lines = cv2.HoughLinesP(cropped_edges, rho=1, theta=np.pi/180, threshold=50, minLineLength=50, maxLineGap=150)
    
    
    if lines is not None:
        # Separate detected lines into left and right based on their slopes
        left_lines = []
        right_lines = []
        
        # Separate lines into left and right based on slope
        for line in lines:
            x1, y1, x2, y2 = line[0]
            slope = (y2 - y1) / float(x2 - x1 + 1e-6)  # Avoid division by zero
            # Negative slope indicates left lane
            if slope < 0:  
                left_lines.append(line[0])
            # Positive slope indicates right lane
            elif slope > 0:  
                right_lines.append(line[0])
        
        # Calculate the average position of left and right lanes
        def average_position(lines):
            x_coords = [x1 + x2 for x1, _, x2, _ in lines]
            return sum(x_coords) / (2 * len(lines)) if lines else None
        
        # Calculate the lane center by averaging left and right lane positions
        left_center = average_position(left_lines)
        right_center = average_position(right_lines)
        
        # Calculate lane center
        if left_center is not None and right_center is not None:
            lane_center = (left_center + right_center) / 2
            # Offset from robot center
            offset = lane_center - TRUCK_POS_X
            return offset
        else:
            rospy.logwarn("One of the lanes is not detected.")
            # Default to no offset if a lane is missing
            return 0
    else:
        rospy.logwarn("No lanes detected.")
        # Default to no offset if no lines are detected
        return 0  

def image_callback(msg):
    """
    ROS callback function for the image topic.
    :param msg: ROS Image message
    """
    try:
        # Convert ROS image message to OpenCV format
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Calculate lane center offset
        offset = calculate_lane_center(cv_image)
        
        # Publish the offset value
        publish_offset(offset)
        
        # Display the processed image for debugging purposes
        cv2.imshow("Lane Detection", cv_image)
        cv2.waitKey(1)
        
    except Exception as e:
        rospy.logerr(f"Error processing image: {e}")

def publish_offset(offset):
    """
    Publish the lane center offset.
    :param offset: Offset value
    """
    # Publishes the computed offset to a ROS topic
    pub = rospy.Publisher('lane_center_offset', Float32, queue_size=1)
    pub.publish(Float32(offset))

def main():
    # Main function for initializing the ROS node and processing images
    rospy.init_node('lane_follower', anonymous=True)
    rospy.Subscriber('/camera/image_raw', Image, image_callback)
    rospy.spin()

if __name__ == "__main__":
    main()