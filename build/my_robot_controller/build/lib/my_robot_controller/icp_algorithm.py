#!/usr/bin/env python3
import open3d as o3d
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
import pandas as pd

class ICPNode(Node):
    def __init__(self):
        super().__init__('icp_node')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.pose_publisher = self.create_publisher(PoseStamped, '/estimated_pose', 10)

        # Inisialisasi referensi scan dan pose global
        self.reference_scan = None
        self.global_pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        self.data_logger = {
            "x" : [],
            "y" : [],
            "yaw" : []
        }

        self.get_logger().info("ICP Localization Node Initialized")

    def scan_callback(self, msg:LaserScan):
        # Konversi LaserScan ke point cloud
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))

        # Filter nilai yang tidak valid
        valid_indices = np.isfinite(ranges) & (ranges > 0)
        ranges = ranges[valid_indices]
        angles = angles[valid_indices]

        # Hitung koordinat kartesian
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        current_scan = np.vstack((x, y)).T
        current_scan = np.hstack((current_scan, np.zeros((current_scan.shape[0], 1))))  # Tambah z=0

        self.get_logger().info(f"Current scan points: {current_scan.shape[0]}")

        if self.reference_scan is not None:
            # Jalankan ICP
            pose = self.perform_icp(self.reference_scan, current_scan)

            if pose is not None:
                # Perbarui pose global
                self.update_global_pose(pose)

                # Publikasikan pose global
                self.publish_pose(self.global_pose)

            # Perbarui referensi
            self.reference_scan = current_scan
        else:
            # Inisialisasi referensi pertama kali
            self.reference_scan = current_scan
            self.get_logger().info("Reference scan initialized.")

    def perform_icp(self, reference, current):
        # Konversi ke Open3D
        source = o3d.geometry.PointCloud()
        target = o3d.geometry.PointCloud()
        source.points = o3d.utility.Vector3dVector(current)
        target.points = o3d.utility.Vector3dVector(reference)

        # Jalankan ICP dengan threshold lebih besar
        threshold = 0.5  # Maksimum jarak
        result = o3d.pipelines.registration.registration_icp(
            source, target, threshold,
            np.eye(4),
            o3d.pipelines.registration.TransformationEstimationPointToPoint()
        )

        # Ambil transformasi
        transform_matrix = result.transformation
        self.get_logger().info(f"Transformation Matrix:\n{transform_matrix}")

        # Jika transformasi kecil (identik dengan matriks identitas), abaikan
        if np.allclose(transform_matrix, np.eye(4)):
            self.get_logger().warning("No significant transformation detected.")
            return None

        # Hitung pose relatif
        x, y = transform_matrix[0, 3], transform_matrix[1, 3]
        yaw = np.arctan2(transform_matrix[1, 0], transform_matrix[0, 0])
        return {"x": x, "y": y, "yaw": yaw}

    def update_global_pose(self, pose):
            dx = pose['x']
            dy = pose['y']
            dtheta = pose['yaw']

            # Update posisi global menggunakan transformasi relatif
            self.global_pose['x'] += dx * np.cos(self.global_pose['yaw']) - dy * np.sin(self.global_pose['yaw'])
            self.global_pose['y'] += dx * np.sin(self.global_pose['yaw']) + dy * np.cos(self.global_pose['yaw'])
            self.global_pose['yaw'] += dtheta

            self.get_logger().info(f"Global Pose - x: {self.global_pose['x']:.3f}, y: {self.global_pose['y']:.3f}, yaw: {self.global_pose['yaw']:.3f}")
            self.data_logger["x"].append(self.global_pose["x"])
            self.data_logger["y"].append(self.global_pose["y"])
            self.data_logger["yaw"].append(self.global_pose["yaw"])


    def publish_pose(self, pose):
        # Buat pesan PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = pose["x"]
        pose_msg.pose.position.y = pose["y"]
        pose_msg.pose.position.z = 0.0

        # Konversi yaw ke quaternion
        qz = np.sin(pose["yaw"] / 2)
        qw = np.cos(pose["yaw"] / 2)
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw

        # Publikasikan pose
        self.pose_publisher.publish(pose_msg)
        self.get_logger().info(f"Published Pose - x: {pose['x']:.3f}, y: {pose['y']:.3f}, yaw: {pose['yaw']:.3f}")

def main(args=None):
    try : 
        rclpy.init(args=args)
        icp_node = ICPNode()
        rclpy.spin(icp_node)
        icp_node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt : 
        dlogger = pd.DataFrame(icp_node.data_logger)
        dlogger.to_csv("data/data_lidar/obstacle/data_raw_lidar_scene_2.csv",index=False)

if __name__ == '__main__':
    main()