#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
import pandas as pd
class PathSaver(Node) : 
    def __init__(self):
        super().__init__('path_saver')
        self.get_logger().info('Path Saver Node has been started & update...')
        self.subs = self.create_subscription(
            Path,
            '/plan',
            self.path_callback,
            10
        )
        self.data_logger = {
            "time": [],
            "x" : [],
            "y" : [],
            "z" : [],
            "qx" : [],
            "qy" : [],
            "qz" : [],
            "qw" : []
        }
        self.path_received = False

    def path_callback(self,msg:Path) : 
        if not self.path_received : 
            # self.get_logger().info('Received path with {} points',len(msg.poses))
            for pose in msg.poses :
                timestamp = msg.header.stamp.sec
                x = pose.pose.position.x
                y = pose.pose.position.y
                z = pose.pose.position.z
                qx = pose.pose.orientation.x
                qy = pose.pose.orientation.y
                qz = pose.pose.orientation.z
                qw = pose.pose.orientation.w
                self.data_logger["x"].append(x)
                self.data_logger["y"].append(y)
                self.data_logger["time"].append(timestamp)
                self.data_logger["z"].append(z)
                self.data_logger["qx"].append(qx)
                self.data_logger["qy"].append(qy)
                self.data_logger["qz"].append(qz)
                self.data_logger["qw"].append(qw)

            self.get_logger().info('Path Saved. Shutting down node')
            self.path_received = True
            rclpy.shutdown()
    
def main(args=None):
    rclpy.init(args=args)
    path_saver = PathSaver()
    try:
        rclpy.spin(path_saver)
    except KeyboardInterrupt:
        print("Thanks")
        Df = pd.DataFrame(path_saver.data_logger)
        Df.to_csv("data/desired_path_scene_1.csv",index=False)
    path_saver.destroy_node()
