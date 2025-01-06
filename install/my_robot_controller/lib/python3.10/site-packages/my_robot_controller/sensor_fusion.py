#!/usr/bin/env python3

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import math

# Main program
if __name__ == "__main__":

    # Load Odometry and IMU data from CSV
    odom_data = pd.read_csv("data/data_odom/data_odom_raw_scene_2.csv")  # File containing odometry data
    imu_data = pd.read_csv("data/data_imu/data_imu_mechanization.csv")    # File containing IMU angular velocity z and orientation
    fusion_data = pd.read_csv("data/data_fusion/ekf_fusion_data_scene_2.csv")
    desired_path  = pd.read_csv("data/desired_path_scene_2.csv")
    yaw_angle = [0]
    x_desired_path = desired_path["x"]
    y_desired_path = desired_path["y"]
    v_desired_x = [0]
    dt = 0.01
    v_desired_y = [0]
    for i in range(1, len(desired_path)) : 
        v_desired_x.append(v_desired_x[i-1] + (x_desired_path[i]-x_desired_path[i-1])/dt)
        v_desired_y.append(v_desired_y[i-1] + (y_desired_path[i]-y_desired_path[i-1])/dt)
        yaw_angle.append(math.atan(v_desired_y[i]/v_desired_x[i]))
    # Extract data
    odom_positions = odom_data[["x", "y"]].values  # [x, y]
    imu_orientations = imu_data["yaw"].values  # yaw
    t_imu = imu_data["time"]
    t_odom = odom_data["time"]
    angular_velocity_z = imu_data["wz"].values  # angular velocity z
    dynamic_imu = pd.read_csv("data/data_fusion/ekf_imu.csv")

    # Store results for visualization
    fused_positions = []
    fused_orientations = []

    # Run EKF
    fused_x = fusion_data["x"]
    fused_y = fusion_data["y"]
    

    # Plot results
    plt.figure(figsize=(10, 6))
    plt.subplot(2,2,1)
    plt.plot(odom_positions[:, 0], odom_positions[:, 1], label="Odometry Position", color="green")
    plt.plot(fused_x,fused_y,label="Fusion",color="blue")
    plt.plot(imu_data["x"],imu_data["y"],label="IMU Position",color="red")
    plt.plot(desired_path["x"],desired_path["y"],label="Desired path")
    plt.title("Position Comparison")
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.legend()
    plt.grid()
    plt.tight_layout()

    # # Plot orientations
    # plt.figure(figsize=(10, 6))
    plt.subplot(2,2,2)
    plt.plot(t_odom,odom_positions[:, 0], label="Odometry Position", color="orange")
    plt.plot(desired_path["x"],label="Desired path")
    plt.plot(fusion_data["time"],fusion_data["x"],label="Fused Position",color="blue")
    plt.plot(t_imu,imu_data["x"],label="IMU Position")
    plt.title("Position-X Comparison")
    plt.xlabel("time (s)")
    plt.ylabel("X (m)")
    plt.legend()
    plt.grid()
    plt.tight_layout()

    # plt.figure(figsize=(10, 6))
    plt.subplot(2,2,3)
    plt.plot(t_odom,odom_positions[:, 1], label="Odometry Position", color="green")
    # plt.plot(t_odom,fused_positions[:, 1], label="Fused Position")
    plt.plot(fusion_data["time"],fusion_data["y"],label="Fused Position",color="blue")
    plt.plot(t_imu,imu_data["y"],label="IMU Position",color="red")
    plt.title("Position-Y Comparison")
    plt.xlabel("time (s)")
    plt.ylabel("Y (m)")
    plt.legend()
    plt.grid()
    plt.tight_layout()

    # plt.figure(figsize=(10, 6))
    plt.subplot(2,2,4)
    # plt.plot(t_imu,imu_orientations,linewidth=2, label="IMU Yaw", color="red")
    plt.plot(yaw_angle)
    # plt.plot(fusion_data['time'],fusion_data["yaw"], label="Fused Yaw", color="blue")
    # plt.plot(t_odom,odom_data["yaw"],label="Odometry")
    plt.title("Orientation Comparison")
    plt.xlabel("Time (s)")
    plt.ylabel("Yaw (rad)")
    plt.legend()
    plt.grid()

    plt.tight_layout()
    plt.show()
