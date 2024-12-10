#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge

class LaneRegressionNode(Node):
    def __init__(self):
        super().__init__('lane_regression')

        # Declare parameters
        self.declare_parameter('picture_width', 640)
        self.declare_parameter('picture_height', 480)
        self.declare_parameter('truck_pos_x', self.get_parameter('picture_width').value // 2)

        # Initialize constants
        self.PICTURE_WIDTH = self.get_parameter('picture_width').value
        self.PICTURE_HEIGHT = self.get_parameter('picture_height').value
        self.TRUCK_POS_X = self.get_parameter('truck_pos_x').value

        # Initialize ROS entities
        self.publisher = self.create_publisher(Float32, 'lane_center_offset', 10)
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

        # Initialize utilities
        self.bridge = CvBridge()
        self.offset_history = []  # For smoothing offset

    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Calculate lane offset
            offset = self.calculate_lane_center(cv_image)
            smoothed_offset = self.smooth_offset(offset)

            # Publish the offset
            self.publisher.publish(Float32(data=smoothed_offset))

            # Display the image (debugging)
            cv2.imshow("Lane Detection", cv_image)
            cv2.waitKey(1)

            # Log the offset
            self.get_logger().info(f"Lane offset: {smoothed_offset}")
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def calculate_lane_center(self, image):
        # Lane detection logic (same as your original implementation)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)

        mask = np.zeros_like(edges)
        roi_corners = np.array([[
            (int(0.1 * self.PICTURE_WIDTH), self.PICTURE_HEIGHT),
            (int(0.9 * self.PICTURE_WIDTH), self.PICTURE_HEIGHT),
            (int(0.55 * self.PICTURE_WIDTH), int(0.5 * self.PICTURE_HEIGHT)),
            (int(0.45 * self.PICTURE_WIDTH), int(0.5 * self.PICTURE_HEIGHT))
        ]], dtype=np.int32)
        cv2.fillPoly(mask, roi_corners, 255)
        cropped_edges = cv2.bitwise_and(edges, mask)

        lines = cv2.HoughLinesP(cropped_edges, 1, np.pi/180, 50, minLineLength=50, maxLineGap=150)

        if lines is not None:
            left_lines, right_lines = [], []
            for line in lines:
                x1, y1, x2, y2 = line[0]
                slope = (y2 - y1) / float(x2 - x1 + 1e-6)
                if slope < 0:
                    left_lines.append(line[0])
                elif slope > 0:
                    right_lines.append(line[0])

            def average_position(lines):
                x_coords = [x1 + x2 for x1, _, x2, _ in lines]
                return sum(x_coords) / (2 * len(lines)) if lines else None

            left_center = average_position(left_lines)
            right_center = average_position(right_lines)

            if left_center is not None and right_center is not None:
                lane_center = (left_center + right_center) / 2
                return lane_center - self.TRUCK_POS_X
        return 0

    def smooth_offset(self, offset):
        self.offset_history.append(offset)
        if len(self.offset_history) > 5:
            self.offset_history.pop(0)
        return sum(self.offset_history) / len(self.offset_history)

def main(args=None):
    rclpy.init(args=args)
    node = LaneRegressionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()