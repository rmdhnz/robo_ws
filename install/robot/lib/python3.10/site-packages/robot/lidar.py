import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import open3d as o3d
o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Error)

class LidarSubscriber(Node):

    def __init__(self):
        super().__init__('lidar_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.previous_cloud = None

    def listener_callback(self, msg):
        current_cloud = self.laserscan_to_pointcloud(msg)
        if current_cloud is None:
            self.get_logger().error('Current cloud is None')
            return
        if self.previous_cloud is not None:
            self.perform_icp(self.previous_cloud, current_cloud)
        self.previous_cloud = current_cloud

    def laserscan_to_pointcloud(self, scan):
        points = []
        angle = scan.angle_min
        for r in scan.ranges:
            if np.isfinite(r) and r < scan.range_max and r > scan.range_min:
                x = r * np.cos(angle)
                y = r * np.sin(angle)
                points.append([x, y, 0])
            angle += scan.angle_increment
        if not points:
            self.get_logger().error('No valid points found in scan')
            return None
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(np.array(points, dtype=np.float32))
        self.get_logger().info(f'Created point cloud with {len(points)} points')
        return cloud

    def preprocess_pointcloud(self, cloud, voxel_size=0.05):
        # Voxel downsampling filter to reduce noise and improve ICP performance
        downsampled_cloud = cloud.voxel_down_sample(voxel_size)
        return downsampled_cloud

    def perform_icp(self, cloud1, cloud2):
        if cloud1.is_empty() or cloud2.is_empty():
            self.get_logger().error('One of the point clouds is empty')
            return
        threshold = 0.02
        trans_init = np.identity(4)
        self.get_logger().info('Starting ICP registration')
        
        # Preprocess point clouds before ICP
        cloud1 = self.preprocess_pointcloud(cloud1)
        cloud2 = self.preprocess_pointcloud(cloud2)
        
        reg_p2p = o3d.pipelines.registration.registration_icp(
            cloud2, cloud1, threshold, trans_init,
            o3d.pipelines.registration.TransformationEstimationPointToPoint())
        
        self.get_logger().info(f'ICP converged: {reg_p2p.fitness > 0.5}, fitness score: {reg_p2p.fitness}')
        self.get_logger().info(f'Transformation matrix: \n{reg_p2p.transformation}')
        
        # Visualize the point clouds and transformation result
        cloud2.transform(reg_p2p.transformation)  # Apply the transformation to cloud2
        o3d.visualization.draw_geometries([cloud1, cloud2])

def main(args=None):
    rclpy.init(args=args)
    lidar_subscriber = LidarSubscriber()
    rclpy.spin(lidar_subscriber)
    lidar_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
