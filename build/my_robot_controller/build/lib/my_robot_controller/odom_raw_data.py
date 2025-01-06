#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import time
import pandas as pd
import numpy as np
import tf_transformations
from std_msgs.msg import String

class PositionOrientationNode(Node):
    def __init__(self):
        super().__init__('odom_raw_data')
        self.get_logger().info('Odom raw has been started')
        self.nav_info_subs = self.create_subscription(
            String,
            '/nav_info',  # Ganti dengan topik yang sesuai
            self.nav_info_callback,
            10  # QoS
        )
        self.still_navigate  = True
        # Subscribing ke topik odom
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',  # Ganti dengan topik yang sesuai
            self.odom_callback,
            10  # QoS
        )
        self.data_logger = {
            "time" : [],
            "x" : [],
            "y" : [],
            "z" : [],
            "or_x" : [],
            "or_y" : [],
            "or_z" : [],
            "or_w" : [],
            "roll" : [],
            "pitch" : [],
            "yaw" : [],
            "vx" : [],
            "vy" : [],
            "vz" : [],
            "wx" : [],
            "wy" : [],
            "wz" : [],
        }
        # Inisialisasi variabel untuk menyimpan data
        self.position = None
        self.orientation = None
        self.current_time = 0
        
        # Timer untuk pengambilan data setiap 0.01 detik
        self.timer_period = 0.01  # 10 ms
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def nav_info_callback(self, msg:String):
        # Jika terdeteksi bahwa robot sedang berjalan, maka mulai mengambil data odom
        self.still_navigate = False if msg.data == "Success" else True
    def odom_callback(self, msg:Odometry):
        # Ambil data posisi dan orientasi dari pesan odom
        self.position = msg.pose.pose.position
        self.orientation = msg.pose.pose.orientation
        self.linear_vel = msg.twist.twist.linear
        self.angular_vel = msg.twist.twist.angular

    def timer_callback(self):
        # Hanya log data jika posisi dan orientasi telah diperbarui
        if self.position and self.orientation and self.current_time <=75.0:
            quaternion = np.array([
                self.orientation.x,
                self.orientation.y,
                self.orientation.z,
                self.orientation.w
            ])
            roll,pitch,yaw = tf_transformations.euler_from_quaternion(quaternion)

            self.get_logger().info(
                f'Position: x={5+self.position.x:.2f}, y={-5+self.position.y:.2f}, z={self.position.z:.2f} | '
                f'Orientation: x={self.orientation.x:.2f}, y={self.orientation.y:.2f}, '
                f'z={self.orientation.z:.2f}, w={self.orientation.w:.2f}'
            )
            self.data_logger["x"].append(5+self.position.x)
            self.data_logger["y"].append(-5+self.position.y)
            self.data_logger["z"].append(self.position.z)
            self.data_logger["or_x"].append(self.orientation.x)
            self.data_logger["or_y"].append(self.orientation.y)
            self.data_logger["or_z"].append(self.orientation.z)
            self.data_logger["or_w"].append(self.orientation.w)
            self.data_logger["roll"].append(roll)
            self.data_logger["pitch"].append(pitch)
            vx = self.linear_vel.x
            vy = self.linear_vel.y
            vz = self.linear_vel.z
            wx = self.angular_vel.x
            wy = self.angular_vel.y
            wz = self.angular_vel.z
            self.data_logger["vx"].append(vx)
            self.data_logger["vy"].append(vy)
            self.data_logger["vz"].append(vz)
            self.data_logger["wx"].append(wx)
            self.data_logger["wy"].append(wy)
            self.data_logger["wz"].append(wz)
            self.data_logger["yaw"].append(yaw)
            self.data_logger["time"].append(self.current_time)
            self.current_time+=self.timer_period

def main(args=None):
    try:
        rclpy.init(args=args)
        node = PositionOrientationNode()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        dlogger = pd.DataFrame(node.data_logger)
        dlogger.to_csv("data/data_lidar/obstacle/data_lidar_raw_scene_2.csv",index=False)
        print("\nFile has been saved!")

