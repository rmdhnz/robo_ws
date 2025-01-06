#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class DynamicObstacleMonitor(Node):
    def __init__(self):
        super().__init__('obstacle_monitor')
        self.subscription = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.get_logger().info('Obstacle monitor has been started & update ...')

    def lidar_callback(self, msg:LaserScan):
        ranges = msg.ranges
        min_distance = min(ranges)
        self.get_logger().info(f"Closest obstacle at {min_distance:.2f} meters")

def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstacleMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Thanks")

if __name__ == '__main__':
    main()
