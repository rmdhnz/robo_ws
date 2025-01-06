#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import tf_transformations

class OdometryNode(Node) : 
    def __init__(self):
        super().__init__('odom_node')
        self.get_logger().info('Odometry node has been started & update...')
        self.subs = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.data_logger = {
            "x" : [],
            "y" : [],
            "yaw" : [],
        }

    
    def odom_callback(self,msg:Odometry) : 
        x = msg.pose.pose.position.x + 5
        y = msg.pose.pose.position.y - 5
        quaternion = np.array([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])
        roll,pitch,yaw = tf_transformations.euler_from_quaternion(quaternion)
        self.get_logger().info(f'Estimated Position\nx:{x:.2f}\ny:{y:.2f}\nyaw: {yaw}')
        self.data_logger["x"].append(x)
        self.data_logger["y"].append(y)
        self.data_logger["yaw"].append(yaw)

def main(args=None):
    try:
        rclpy.init(args=args)
        mynode = OdometryNode()
        rclpy.spin(mynode)
        rclpy.shutdown()
    except KeyboardInterrupt:
        Df = pd.DataFrame(mynode.data_logger)
        Df.to_csv("odometry_data_scene_1.csv", index=False)
        # plt.figure(1)
        # plt.plot(mynode.data_logger["x"],mynode.data_logger["y"],)
        # plt.title("Data posisi Odometry")
        # plt.xlabel("x-axis")
        # plt.ylabel("y-axis")
        # plt.grid(True)
        
        # plt.figure(2)
        # plt.subplot(3,1,1)
        # plt.plot(mynode.data_logger["x"])
        # plt.grid(True)
        # plt.xlabel("time")
        # plt.ylabel("x-position")
        # plt.subplot(3,1,2)
        # plt.plot(mynode.data_logger["y"])
        # plt.xlabel("time")
        # plt.grid(True)
        # plt.ylabel("y-position")
        # plt.subplot(3,1,3)
        # plt.plot(mynode.data_logger["yaw"])
        # plt.xlabel("time")
        # plt.ylabel("yaw-position")
        # plt.grid(True)

        # plt.show()
        


""" kalau begitu, sekarang berikan algoritma perhitungan extended kalman filter, lengkap dengan penjelasan data apa saja yang diperlukan, model sistem, pengukuran, fokus utama nya adalah untuk estimasi posisi dan orientasi """