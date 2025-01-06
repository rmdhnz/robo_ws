#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
from scipy.spatial.transform import Rotation as R
import tf_transformations
import matplotlib.pyplot as plt
import pandas as pd
class IMUPositionEstimator(Node):
    def __init__(self):
        super().__init__('imu_node')
        self.est_x = []
        self.est_y = []
        self.est_yaw = []
        self.data_logger = {
            "x"  :[], 
            "y" : [],
            "yaw" : [],
            "vx" : [],
            "vy" : [],
            "wx" : [],
            "wy" : [],
            "wz" : [],
        }
        self.get_logger().info('Imu Node has been started & updating...')

        # Inisialisasi variabel kecepatan dan posisi
        self.velocity = np.array([0.0, 0.0, 0.0])  # vx, vy, vz
        self.position = np.array([0.0, 0.0, 0.0])  # px, py, pz

        # Waktu terakhir pembacaan
        self.last_time = None

        # Subscribe ke topik IMU
        self.subscription = self.create_subscription(
            Imu,
            '/imu',  # Gantilah ini dengan topik IMU-mu
            self.imu_callback,
            10
        )


    def imu_callback(self, msg:Imu):
        # Mendapatkan waktu sekarang
        current_time = self.get_clock().now().nanoseconds / 1e9  # Konversi ke detik
        # Jika ini pembacaan pertama, simpan waktu dan keluar
        if self.last_time is None:
            self.last_time = current_time
            return

        # Delta waktu (dt)
        dt = current_time - self.last_time
        self.last_time = current_time

        # Mendapatkan percepatan linear dari IMU dalam kerangka lokal
        linear_acceleration = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])

        # Mendapatkan orientasi dalam bentuk quaternion dan mengonversinya ke rotasi matrix
        quaternion = np.array([
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ])
        roll,pitch,yaw = tf_transformations.euler_from_quaternion(quaternion)

        
        rotation_matrix = R.from_quat(quaternion).as_matrix()



        # Konversi percepatan ke kerangka dunia
        acceleration_world = rotation_matrix.dot(linear_acceleration)

        # Menghapus efek gravitasi di sumbu Z
        acceleration_world[2] -= 9.81

        # Integrasi pertama: memperbarui kecepatan
        self.velocity += acceleration_world * dt

        # Integrasi kedua: memperbarui posisi
        self.position += self.velocity * dt
        self.data_logger["vx"].append(self.velocity[0])
        self.data_logger["vy"].append(self.velocity[1])
        self.data_logger["yaw"].append(yaw)
        self.data_logger["wx"].append(msg.angular_velocity.x)
        self.data_logger["wy"].append(msg.angular_velocity.y)
        self.data_logger["wz"].append(msg.angular_velocity.z)


        # Menampilkan posisi yang diperkirakan
        self.get_logger().info(f"Posisi: x={self.position[0]:.2f}, y={self.position[1]:.2f}, yaw={yaw}")
        self.est_x.append(self.position[0])
        self.est_y.append(self.position[1])
        self.est_yaw.append(yaw)
        self.data_logger["x"].append(self.position[0])
        self.data_logger["y"].append(self.position[1])

def main(args=None):
    try : 
        rclpy.init(args=args)
        node = IMUPositionEstimator()
        rclpy.spin(node)
        rclpy.shutdown()
    except KeyboardInterrupt :
        D_logger = pd.DataFrame(node.data_logger)
        D_logger.to_csv("data/imu_data_scene_1.csv")
        plt.figure(1)
        plt.plot(node.data_logger["x"],node.data_logger["y"])
        plt.title("Data posisi X,Y")
        plt.xlabel("X-position")
        plt.ylabel("Y-position")
        plt.grid(True)
        plt.show()
        # D_logger.to_csv("imu_data_test.csv",index=False)
        # plt.figure(1)
        # plt.subplot(3,1,1)
        # plt.plot(node.est_x)
        # plt.grid(True)
        # plt.title("Data posisi X")
        # plt.xlabel("Time")
        # plt.ylabel("Posisi")
        # plt.subplot(3,1,2)
        # plt.plot(node.est_y)
        # plt.title("Data posisi Y")
        # plt.grid(True)
        # plt.xlabel("Time")
        # plt.ylabel("Posisi")
        # plt.subplot(3,1,3)
        # plt.title("Data orientasi Yaw")
        # plt.plot(node.est_yaw)
        # plt.grid(True)
        # plt.xlabel("Time")
        # plt.ylabel("Sudut")
        # # plt.show()
        # plt.figure(2)
        # plt.plot(node.est_x,node.est_y)
        # plt.xlabel("X-axis")
        # plt.ylabel("Y-axis")
        # plt.show()

        # node.get_logger().info(f"MSE (X) : {np.mean(node.est_x):.4f}")
        # node.get_logger().info(f"MSE (Y) : {np.mean(node.est_y):.4f}")

if __name__ == '__main__':
    main()
