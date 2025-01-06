#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
import matplotlib.pyplot as plt
class PathListener(Node):
    def __init__(self):
        super().__init__('path_listener')
        self.get_logger().info('Path listener has been started...')
        self.subs = self.create_subscription(
            Path,
            '/plan',
            self.path_callback,
            10
        )
        self.x_pos = []
        self.y_pos = []
    
    def path_callback(self, msg:Path):
        for pose in msg.poses : 
            x = pose.pose.position.x
            y = pose.pose.position.y
            self.get_logger().info(f'X = {x}')
            self.get_logger().info(f'Y = {y}')
            self.x_pos.append(x)
            self.y_pos.append(y)


def main(args=None):
    try:
        rclpy.init(args=args)
        mynode = PathListener()
        rclpy.spin(mynode)
        rclpy.shutdown()
    except KeyboardInterrupt:
        plt.plot(mynode.x_pos,mynode.y_pos)
        plt.show()