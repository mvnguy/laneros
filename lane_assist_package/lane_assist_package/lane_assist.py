#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

# Parameter for P-Controller
Kp = -4

class LaneAssistNode(Node):

    def __init__(self):
        super().__init__('lane_assist')
        self.subscription = self.create_subscription(Float32, 'laneregression_offset', self.callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription  # prevent unused variable warning

    def callback(self, msg):
        try:
            offset = msg.data

            # P-Controller
            steering_angle = Kp * offset

            # Create Twist message for TurtleBot
            twist = Twist()
            twist.linear.x = 0.2  # Constant forward speed
            twist.angular.z = steering_angle  # Adjust turning based on lane offset

            # Publish Twist message
            self.publisher.publish(twist)

            self.get_logger().info(f'Steering Angle: {steering_angle}')

        except Exception as e:
            twist = Twist()  # Stop the robot
            self.publisher.publish(twist)
            self.get_logger().error(f'Error in callback: {e}')

def main(args=None):
    rclpy.init(args=args)

    lane_assist_node = LaneAssistNode()

    try:
        rclpy.spin(lane_assist_node)
    except KeyboardInterrupt:
        pass

    lane_assist_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
