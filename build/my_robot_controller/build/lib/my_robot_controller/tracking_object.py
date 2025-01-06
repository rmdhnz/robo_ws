#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import pandas as pd

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('tracking_object')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.get_logger().info("Object Detection Node Initialized")
        self.closest_obj = {
            "x" : [],
            "y": [],
        }

    def scan_callback(self, msg: LaserScan):
        # Konversi data polar LiDAR ke Kartesian
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))

        # Filter data valid (hilangkan nilai inf atau NaN)
        valid_indices = np.isfinite(ranges) & (ranges > 0)
        ranges = ranges[valid_indices]
        angles = angles[valid_indices]

        # Konversi ke koordinat Kartesian
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)

        # Gabungkan menjadi array posisi objek
        positions = np.vstack((x, y)).T

        # Log posisi objek
        self.get_logger().info(f"Detected objects (relative positions):\n{positions}")

        # Jika diperlukan, pilih objek terdekat
        closest_object_index = np.argmin(ranges)
        closest_object = positions[closest_object_index]
        self.get_logger().info(f"Closest object at: x={closest_object[0]:.2f}, y={closest_object[1]:.2f}")
        self.closest_obj["x"].append(closest_object[0])
        self.closest_obj["y"].append(closest_object[1])

def main(args=None):
    try:
        rclpy.init(args=args)
        object_detection_node = ObjectDetectionNode()
        rclpy.spin(object_detection_node)
        object_detection_node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        data_frame = pd.DataFrame(object_detection_node.closest_obj)
        data_frame.to_csv("data/data_lidar/object_tracking_with_obs.csv")
        print("File has been saved successfully!")
        

if __name__ == '__main__':
    main()
