#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import csv
from datetime import datetime
import signal
import sys

class ImuDataSaver(Node):
    def __init__(self):
        super().__init__('imu_data_saver')
        self.subscription = self.create_subscription(
            Imu,
            '/bno055/imu',
            self.imu_callback,
            10)
        self.subscription
        
        self.csv_file = open('imu_data.csv', mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        self.csv_writer.writerow([
            'time', 
            'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w',
            'angular_velocity_x', 'angular_velocity_y', 'angular_velocity_z',
            'linear_acceleration_x', 'linear_acceleration_y', 'linear_acceleration_z'
        ])

    def imu_callback(self, msg):
        current_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')

        orientation_x = msg.orientation.x
        orientation_y = msg.orientation.y
        orientation_z = msg.orientation.z
        orientation_w = msg.orientation.w

        angular_velocity_x = msg.angular_velocity.x
        angular_velocity_y = msg.angular_velocity.y
        angular_velocity_z = msg.angular_velocity.z

        linear_acceleration_x = msg.linear_acceleration.x
        linear_acceleration_y = msg.linear_acceleration.y
        linear_acceleration_z = msg.linear_acceleration.z

        self.csv_writer.writerow([
            current_time,
            orientation_x, orientation_y, orientation_z, orientation_w,
            angular_velocity_x, angular_velocity_y, angular_velocity_z,
            linear_acceleration_x, linear_acceleration_y, linear_acceleration_z
        ])

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()

def signal_handler(sig, frame):
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    global node
    node = ImuDataSaver()

    signal.signal(signal.SIGINT, signal_handler)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()