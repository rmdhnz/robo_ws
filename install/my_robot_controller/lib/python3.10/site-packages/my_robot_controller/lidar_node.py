#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import pandas as pd
class LidarToCartesian(Node):
    def __init__(self):
        super().__init__('lidar_node')
        self.get_logger().info('Lidar node has been started &  updating...')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan', 
            self.scan_callback,
            10
        )
        self.data_logger = {
            "angle_min" : [],
            "angle_max" : [],
            "angle_increment" : [],
            "time_increment" : [],
            "scan_time" : [],
            "range_min" : [],
            "range_max" : [],
            "ranges" : [],
            "intensities" : []  # Daftar koordinat kartesian
        } 
        self.timer_period = 0.01
        self.timer = self.create_timer(self.timer_period,self.timer_callback)
        self.current_time = 0
        self.latest_lidar_data = None
    
    def timer_callback(self):
        if self.latest_lidar_data is not None and self.current_time <= 60.0:
            angle_min = self.latest_lidar_data.angle_min      # Sudut minimum
            self.data_logger["angle_min"].append(angle_min)
            self.data_logger["angle_max"].append(self.latest_lidar_data.angle_max)
            ranges = np.array(self.latest_lidar_data.ranges)
            angle_increment = self.latest_lidar_data.angle_increment 
            self.data_logger["angle_increment"].append(angle_increment)
            self.data_logger["time_increment"].append(self.latest_lidar_data.time_increment)
            self.data_logger["scan_time"].append(self.latest_lidar_data.scan_time)
            self.data_logger["range_min"].append(self.latest_lidar_data.range_min)
            self.data_logger["range_max"].append(self.latest_lidar_data.range_max)
            self.data_logger["ranges"].append(ranges)

            # Mendapatkan parameter sudut dari pesan LaserScan
            self.data_logger["intensities"].append(self.latest_lidar_data.intensities) 
            self.current_time+=self.timer_period
            self.get_logger().info('Data logging...')

    def scan_callback(self, msg:LaserScan):
        self.latest_lidar_data = msg
        # Mendapatkan data jarak dari Lidar
def main(args=None):
    try : 
        rclpy.init(args=args)
        node = LidarToCartesian()
        rclpy.spin(node)  # Jalankan node sampai dihentikan
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt : 
        dlogger = pd.DataFrame(node.data_logger)
        dlogger.to_csv("data/data_lidar/data_raw_lidar_diam.csv",index=False)

if __name__ == '__main__':
    main()
