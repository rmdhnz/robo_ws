#/!usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import tf_transformations
import math
import pandas as pd
from scipy.spatial.transform import Rotation as R
class ImuSubscriber(Node):
    def __init__(self):
        super().__init__('imu_raw_data')
        self.get_logger().info('Imu Raw Data has been started & update...')
        # Subscribe ke topik /imu
        self.subscription = self.create_subscription(
            Imu,
            '/bno055/imu',
            self.imu_callback,
            10  # QoS depth
        )
        self.subscription  # prevent unused variable warning

        # Timer untuk menerima data setiap 0.01 detik
        self.current_time = 0
        self.timer_period = 0.01  # 10 ms
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.latest_imu_data = None
        self.data_logger = {
            "time" : [],
            "ax" : [],
            "ay" : [],
            "az" : [],
            "wx" : [],
            "wy" : [],
            "wz" : [],
            "or_x" : [],
            "or_y" : [],
            "or_z" : [],
            "or_w" : [],
        }

    def imu_callback(self, msg:Imu):
        # Simpan data IMU terbaru dari callback
        self.latest_imu_data = msg

    def timer_callback(self):
        if self.latest_imu_data is not None:
            # Ambil data dari IMU
            linear_acceleration = self.latest_imu_data.linear_acceleration
            angular_velocity = self.latest_imu_data.angular_velocity
            orientation_quaternion = self.latest_imu_data.orientation

            # Konversi orientasi dari quaternion ke Euler
            quaternion = (
                orientation_quaternion.x,
                orientation_quaternion.y,
                orientation_quaternion.z,
                orientation_quaternion.w,
            )
            euler = tf_transformations.euler_from_quaternion(quaternion)
            self.data_logger["or_x"].append(quaternion[0])
            self.data_logger["or_y"].append(quaternion[1])
            self.data_logger["or_z"].append(quaternion[2])
            self.data_logger["or_w"].append(quaternion[3])

            # Tampilkan data
            self.get_logger().info(
                f"Linear Acceleration: x={linear_acceleration.x:.2f}, "
                f"y={linear_acceleration.y:.2f}, z={linear_acceleration.z:.2f}"
            )
            self.get_logger().info(
                f"Angular Velocity: x={angular_velocity.x:.2f}, "
                f"y={angular_velocity.y:.2f}, z={angular_velocity.z:.2f}"
            )
            self.data_logger["ax"].append(linear_acceleration.x)
            self.data_logger["ay"].append(linear_acceleration.y)
            self.data_logger["az"].append(linear_acceleration.z)
            self.data_logger["wx"].append(angular_velocity.x)
            self.data_logger["wy"].append(angular_velocity.y)
            self.data_logger["wz"].append(angular_velocity.y)
            self.data_logger["time"].append(self.current_time)
            self.current_time += self.timer_period

def main(args=None):
    try:
        rclpy.init(args=args)
        imu_subscriber = ImuSubscriber()
        rclpy.spin(imu_subscriber)

        imu_subscriber.destroy_node()
        rclpy.shutdown()        
    except KeyboardInterrupt:
        dlogger = pd.DataFrame(imu_subscriber.data_logger)
        dlogger.to_csv("data/data_imu/data_impl_scene_1.csv",index=False)


if __name__ == '__main__':
    main()
