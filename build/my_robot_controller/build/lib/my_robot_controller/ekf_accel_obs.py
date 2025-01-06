#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
class ExtendedKalmanFilter:
    def __init__(self, Q, R):
        # State vector [a_x, a_y, a_z]
        self.x = np.zeros((3, 1))
        # Covariance matrix
        self.P = np.eye(3)
        # Process noise covariance
        self.Q = Q
        # Measurement noise covariance
        self.R = R

    def predict(self, A):
        # Predict state (f(x) = x, so no change in state)
        self.x = self.x
        # Predict covariance
        self.P = A @ self.P @ A.T + self.Q

    def update(self, z, H):
        # Measurement residual
        y = z - self.x
        # Measurement covariance
        S = H @ self.P @ H.T + self.R
        # Kalman Gain
        K = self.P @ H.T @ np.linalg.inv(S)
        # Update state
        self.x = self.x + K @ y
        # Update covariance
        self.P = (np.eye(3) - K @ H) @ self.P

# Simulasi data percepatan linear dengan noise
data = pd.read_csv("data/data_imu/obstacle/data_raw_imu_scene_2.csv")
np.random.seed(42)
true_acceleration = np.array([[0], [0], [0]])  # True acceleration (robot is stationary)
measurement_variance = 0.05  # Variance in accelerometer measurements
bias_ax = -0.065
bias_ay = 0.00017
bias_az = 0.00012
data["ax"]-=bias_ax
data["ay"]-=bias_ay
data["az"]-=bias_az
accel_measurements = data[['ax', 'ay', 'az']].to_numpy()
# Inisialisasi EKF
Q = np.eye(3) * 0.01  # Process noise covariance
R = np.eye(3) * measurement_variance  # Measurement noise covariance
ekf = ExtendedKalmanFilter(Q, R)
# Matriks A dan H
A = np.eye(3)  # State transition Jacobian
H = np.eye(3)  # Measurement Jacobian

# Estimasi EKF
estimates = []
try :
    for z in accel_measurements:
        ekf.predict(A)
        ekf.update(z, H)
        estimates.append(ekf.x.flatten())

    estimates = np.array(estimates)
    kf_ax =  []
    kf_ay =  []
    kf_az =  []

    for i in range(len(estimates)) : 
        kf_ax.append(estimates[i][0])
        kf_ay.append(estimates[i][1])
        kf_az.append(estimates[i][2])
    # Plot hasil
    time = data["time"] # Simulate for 60 seconds
    plt.figure(figsize=(10, 8))
    plt.subplot(3,1,1)
    plt.title("error Measured vs Filtered Acceleration -X")
    plt.plot(time,1*(accel_measurements[:,0]),label="Noisy Measurement X")
    plt.plot(time,1*(estimates[:,0]),label="Filtered Accelero-X")
    plt.xlabel('Time (s)')
    plt.ylabel('X Acceleration (m/s²)')
    plt.legend()
    plt.subplot(3,1,2)
    plt.title("error Measured vs Filtered Acceleration - Y")
    plt.plot(time, 1*(accel_measurements[:,1]),label="Noisy Measurement Y")
    plt.plot(time, 1*(estimates[:,1]),label="Filtered Accelero-Y")
    plt.xlabel('Time (s)')
    plt.ylabel('Y Acceleration (m/s²)')
    plt.legend()
    plt.subplot(3,1,3)
    plt.title("error Measured vs Filtered Acceleration - Z")
    plt.plot(time, 1*(accel_measurements[:,2]),label="Noisy Measurement Z")
    plt.plot(time, 1*(estimates[:,2]),label="Filtered Accelero-Z")
    plt.xlabel('Time (s)')
    plt.ylabel('Z Acceleration (m/s²)')
    plt.legend()

    plt.tight_layout()
    plt.show()
    # print("Error Ax : ",err_ax:=np.mean(np.abs(data["ax"])))
    # print("Error Filtered Ax : ",err_kf_ax:=np.mean(np.abs(estimates[:,0])))
    # print("Error Ay : ",err_ay:=np.mean(np.abs(data["ax"])))
    # print("Error Filtered Ay : ",err_kf_ay:=np.mean(np.abs(estimates[:,1])))
    # print("Error Az : ",err_az:=np.mean(np.abs(data["az"])))
    # print("Error Filtered Az : ",err_kf_az:=np.mean(np.abs(estimates[:,2])))
    # print("Ax :Error Filtered lebih kecil " if err_ax > err_kf_ax else "Ax: Error Filtered lebih besar")
    # print("Ay: Error Filtered lebih kecil " if err_ay > err_kf_ay else "Ay: Error Filtered lebih besar")
    # print("Az: Error Filtered lebih kecil" if err_az > err_kf_az else "Az: Error Filtered lebih besar")


    dlogger = {
            "time" : data["time"],
            "ax" : data["ax"],
            "ay" : data["ay"],
            "az" : data["az"],
            "wx" : data["wx"],
            "wy" : data["wy"],
            "wz" : data["wz"],
            "or_x" : data["or_x"],
            "or_y" : data["or_y"],
            "or_z" : data["or_z"],
            "or_w" : data["or_w"],
            "kf_ax" :  kf_ax,
            "kf_ay" :  kf_ay,
            "kf_az" :  kf_az,
    }
    d = pd.DataFrame(dlogger)
    d.to_csv("data/data_imu/data_ekf_accel.csv",index=False)
except Exception as e:
    print("Error: " + str(e))
