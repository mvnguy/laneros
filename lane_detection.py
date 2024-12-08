#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point
from lane_keeping_assist.msg import ClusterData
import os

class LaneDetectionDummy(Node):

    def __init__(self):
        super().__init__('lanedetection_dummy')
        self.publisher = self.create_publisher(ClusterData, 'cluster_data', 10)
        self.timer = self.create_timer(2.0, self.publish_cluster_data)  # 0.5 Hz
        self.file_path = "/home/zflab/catkin_ws/src/laneregression/scripts/all_cluster"

    def publish_cluster_data(self):
        try:
            with open(self.file_path, "r") as file_cluster:
                for line in file_cluster:
                    all_cluster = ClusterData()
                    string_cluster = line.split(";")

                    for i in range(len(string_cluster) - 1):
                        string_points = string_cluster[i].split(",")

                        all_cluster.size.append(int((len(string_points) - 1) / 2))

                        for j in range(0, len(string_points) - 1, 2):
                            single_point = Point()
                            single_point.x = int(string_points[j])
                            single_point.y = int(string_points[j+1])
                            single_point.z = 0
                            all_cluster.points.append(single_point)

                    self.publisher.publish(all_cluster)
                    self.get_logger().info(f'Published cluster data: {all_cluster.size}')

        except Exception as e:
            self.get_logger().error(f'Error reading or publishing cluster data: {e}')

def main(args=None):
    rclpy.init(args=args)

    lane_detection_dummy = LaneDetectionDummy()

    try:
        rclpy.spin(lane_detection_dummy)
    except KeyboardInterrupt:
        pass

    lane_detection_dummy.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
