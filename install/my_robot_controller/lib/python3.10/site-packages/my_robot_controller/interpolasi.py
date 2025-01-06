#!/usr/bin/env python3
import tf_transformations
import pandas as pd
import numpy as np
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
import math
# Load reference path from CSV

def quaternion_to_yaw(qx, qy, qz, qw):
    """
    Convert quaternion to yaw (rotation around Z-axis).
    Args:
        qx, qy, qz, qw: Components of the quaternion.
    Returns:
        float: Yaw angle in radians.
    """
    _, _, yaw = tf_transformations.euler_from_quaternion([qx, qy, qz, qw])
    return yaw

reference_path = pd.read_csv("data/desired_path_scene_2.csv")  # Ganti dengan nama file Anda
fusion_data = pd.read_csv("data/data_fusion/obstacle/ekf_fusion_data_scene_2.csv")
imu_data = pd.read_csv("data/data_imu/data_imu_mechanization.csv") 
odom_data = pd.read_csv("data/data_odom/obstacle/data_odom_raw_scene_2.csv")
lidar = pd.read_csv("data/data_lidar/obstacle/data_raw_lidar_scene_2.csv")
# fusion_gabungan = pd.read_csv("data/data_fusion/ekf_fusion_data_scene_3_in_ex.csv")
reference_path = reference_path.sort_values(by=['x', 'y']).reset_index(drop=True)
# Extract x and y coordinates
x_coords = reference_path['x'].values
y_coords = reference_path['y'].values
lidar_x = lidar["x"].values
lidar_y = lidar["y"].values
# Calculate cumulative distances along the path
distances = np.cumsum(np.sqrt(np.diff(x_coords, prepend=x_coords[0])**2 +
                              np.diff(y_coords, prepend=y_coords[0])**2))

distances_lidar = np.cumsum(np.sqrt(np.diff(lidar_x, prepend=lidar_x[0])**2 +
                              np.diff(lidar_y, prepend=lidar_y[0])**2))
# Create interpolation functions for x and y
interp_x = interp1d(distances, x_coords, kind='linear')
interp_y = interp1d(distances, y_coords, kind='linear')
interp_lidar_x = interp1d(distances_lidar, lidar_x, kind='linear')
interp_lidar_y = interp1d(distances_lidar, lidar_y, kind='linear')

# Generate evenly spaced distances
target_points = len(imu_data)  # Jumlah poin yang diinginkan
target_distances = np.linspace(0, distances[-1], target_points)
target_distances_lidar = np.linspace(0,distances_lidar[-1],target_points)

# Interpolate x and y coordinates
interpolated_x = interp_x(target_distances)
interpolated_y = interp_y(target_distances)
interpolated_lidar_x = interp_lidar_x(target_distances_lidar)
interpolated_lidar_y = interp_lidar_y(target_distances_lidar)
# Save interpolated path to CSV
interpolated_path = pd.DataFrame({
    'x': interpolated_x,
    'y': interpolated_y,
})
interpolated_lidar = pd.DataFrame({
    "x" : interpolated_lidar_x,
    "y" : interpolated_lidar_y
})
interpolated_path.to_csv("interpolated_path.csv", index=False)
yaw_angle = [0]
v_desired_x = [0]
dt = 0.01
v_desired_y = [0]
for i in range(1, len(interpolated_x)) : 
        v_desired_x.append(v_desired_x[i-1] + (interpolated_x[i]-interpolated_x[i-1])/dt)
        v_desired_y.append(v_desired_y[i-1] + (interpolated_y[i]-interpolated_y[i-1])/dt)
        yaw_angle.append(math.atan(v_desired_y[i]/v_desired_x[i]))
print("Jumlah data interpolated :",len(interpolated_path))
print("Jumlah data fusion:",len(fusion_data))
print(interpolated_x)
mse_x_fusion = np.mean((interpolated_path['x'] - fusion_data['x'])**2)
mse_y_fusion = np.mean((interpolated_path['y'] - fusion_data['y'])**2)
mse_x_odometry = np.mean((interpolated_path['x'] - odom_data['x'])**2)
mse_y_odometry = np.mean((interpolated_path['y'] - odom_data['y'])**2)
mse_x_imu = np.mean((interpolated_path['x'] - imu_data['x'])**2)
mse_y_imu = np.mean((interpolated_path['y'] - imu_data['y'])**2)
mse_lidar_x = np.mean((interpolated_lidar_x - odom_data["x"])**2)
mse_lidar_y = np.mean((interpolated_lidar_y - odom_data["y"])**2)
# mse_y_gabungan = mse_y_imu = np.mean((interpolated_path['y'] - fusion_gabungan['y'])**2)
# mse_x_gabungan = mse_x_imu = np.mean((interpolated_path['x'] - fusion_gabungan['x'])**2)
#print semua mse
print(f"MSE x Fusion: {mse_x_fusion}")
print(f"MSE y Fusion: {mse_y_fusion}")
# print(f"MSE x Fusion (Gabungan): {mse_x_gabungan}")
# print(f"MSE y Fusion(Gabungan): {mse_y_gabungan}")
print(f"MSE x Odometry: {mse_x_odometry}")
print(f"MSE y Odometry: {mse_y_odometry}")
print(f"MSE x IMU: {mse_x_imu}")
print(f"MSE y IMU: {mse_y_imu}")
print(f"MSE ICP -x = {mse_lidar_x}")
print(f"MSE ICP -y = {mse_lidar_y}")
# Plot original and interpolated paths
theta = np.linspace( 0 , 2 * np.pi , 150 )
radius = 0.6
obs_x = radius*np.cos(theta) + 5
obs_y = radius*np.sin(theta) - 2
obs_x_1 = 0.4*np.cos(theta) + 8
obs_y_1 = 0.4*np.sin(theta) - 4

plt.figure(figsize=(10, 6))
# plt.subplot(2,2,1)
plt.plot(fusion_data['x'], fusion_data['y'],label="Fusion data (Internal)",color="blue",linewidth="3")
plt.plot(odom_data['x'], odom_data['y'],label="RG",color="red")
plt.plot(interpolated_lidar_x, interpolated_lidar_y,label="ICP",color="green")
# plt.plot(lidar['x'], lidar['y'],label="ICP")
plt.plot(obs_x,obs_y,label="obstacle",color="black",linestyle="dashed")
plt.plot(obs_x_1,obs_y_1,color="black",linestyle="dashed")
# plt.plot(fusion_gabungan['x'], fusion_gabungan['y'], label="Fusion (Gabungan)")
plt.xlabel("X")
plt.ylabel("Y")
plt.legend()
plt.title("Position Comparison")
plt.grid()
plt.show()
exit()
plt.subplot(2,2,2)
plt.plot(fusion_data['x'], label="Fusion data (Internal)")
plt.plot(odom_data['x'],label="Odometry data")
plt.plot(imu_data['x'], label="IMU data")
plt.plot(interpolated_x,'-', label="RG")
# plt.plot(fusion_gabungan['x'], label="Fusion (Gabungan)")
plt.xlabel("X")
plt.ylabel("Y")
plt.legend()
plt.title("Position-X Comparison")
plt.grid()

plt.subplot(2,2,3)
plt.plot( fusion_data['y'],label="Fusion data(internal)")
plt.plot( odom_data['y'],label="Odometry data")
plt.plot( imu_data['y'],label="IMU data")
# plt.plot(fusion_gabungan['y'], label="Fusion (Gabungan)")
plt.plot(interpolated_y, '-', label="RG")
plt.xlabel("X")
plt.ylabel("Y")
plt.legend()
plt.title("Position-Y Comparison")
plt.grid()

plt.subplot(2,2,4)
plt.plot( fusion_data['yaw'],label="Fusion data(internal)")
plt.plot( odom_data['yaw'],label="Odometry data")
plt.plot( imu_data['yaw'],label="IMU data")
# plt.plot(yaw_angle,label="Yaw angle")
# plt.plot(fusion_gabungan['yaw'], label="Fusion (Gabungan)")
plt.xlabel("X")
plt.ylabel("Y")
plt.legend()
plt.title("Yaw Comparison")
plt.grid()
plt.tight_layout()
plt.show()

