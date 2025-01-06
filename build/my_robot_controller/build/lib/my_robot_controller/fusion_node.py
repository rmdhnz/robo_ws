#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import tf_transformations
class FusionNode(Node):
    def __init__(self):
        super().__init__('fusion_node')
        self.get_logger().info('Fusion node has been started ...')
        self.ekf_subs = self.create_subscription(
            Odometry,
            "/odometry/filtered",
            self.ekf_callback,
            10
        )
        self.data_logger = {
            "x" : [],
            "y" : [],
            "yaw" : [],
        }
    
    def ekf_callback(self,msg:Odometry) :  
        x = msg.pose.pose.position.x+5
        y = msg.pose.pose.position.y-5

        quaternion = np.array([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])
        roll,pitch,yaw = tf_transformations.euler_from_quaternion(quaternion)
        self.get_logger().info(f'Filtered Data\nX:{x:.3f}\nY:{y:.3f}\nyaw:{yaw}')
        self.data_logger["x"].append(x)
        self.data_logger["y"].append(y)
        self.data_logger["yaw"].append(yaw)

def main(args=None):
    try:
        rclpy.init(args=args)
        mynode = FusionNode()
        rclpy.spin(mynode)
        rclpy.shutdown()
    except KeyboardInterrupt:
        Df = pd.DataFrame(mynode.data_logger)
        Df.to_csv("fusion_data_logger_scene_1_in_ext.csv", index=False)
        # plt.figure(1)
        # plt.plot(mynode.data_logger["x"],mynode.data_logger["y"])
        # plt.title("Filtered Data")
        # plt.xlabel("X-axis")
        # plt.ylabel("Y-axis")
        # plt.grid(True)
        # plt.show()