#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
import cv2
from cv_bridge import CvBridge

class LaneDetectionNode(Node):
    def __init__(self):
        super().__init__('lane_detection')

        # Subscriber to the camera feed
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

        # Publisher for the lane offset
        self.publisher = self.create_publisher(Float32, 'laneregression_offset', 10)

        # CvBridge for converting ROS images to OpenCV format
        self.bridge = CvBridge()

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Process the image to detect lane offset
            offset = self.detect_lane(cv_image)

            # Publish the offset
            offset_msg = Float32()
            offset_msg.data = float(offset)
            self.publisher.publish(offset_msg)

            self.get_logger().info(f'Lane offset: {offset}')
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def detect_lane(self, image):
        """
        Detect lane markings and calculate the offset.

        Args:
            image: Input OpenCV image.

        Returns:
            Offset from the robot's center to the lane center.
        """
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Apply Gaussian blur
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Perform edge detection
        edges = cv2.Canny(blurred, 50, 150)

        # Define a region of interest (ROI)
        mask = cv2.rectangle(edges, (0, 300), (640, 480), 255, -1)
        cropped = cv2.bitwise_and(edges, mask)

        # Perform Hough Line Transform
        lines = cv2.HoughLinesP(cropped, rho=1, theta=np.pi/180, threshold=50, minLineLength=50, maxLineGap=150)

        if lines is not None:
            # Average the detected lines and calculate offset
            x_positions = [(x1 + x2) / 2 for x1, _, x2, _ in lines[:, 0]]
            lane_center = sum(x_positions) / len(x_positions)
            offset = lane_center - (image.shape[1] / 2)
            return offset
        else:
            self.get_logger().warn("No lane lines detected.")
            return 0.0

def main(args=None):
    rclpy.init(args=args)

    lane_detection_node = LaneDetectionNode()

    try:
        rclpy.spin(lane_detection_node)
    except KeyboardInterrupt:
        pass

    lane_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()