#!/usr/bin/env python3
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

class ExtendedKalmanFilter:
    def __init__(self, Q, R):
        # State vector [omega_x, omega_y, omega_z]
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

# Membaca data dari CSV
data = pd.read_csv('data/data_imu/data_ekf_accel.csv')  # Ganti dengan nama file Anda
bias_wx = -1.068e-05
bias_wy = 0.00312
bias_wz = 0.00312
data["wx"]-=bias_wx
data["wy"]-=bias_wy
data["wz"]-=bias_wz
gyro_measurements = data[['wx', 'wy', 'wz']].to_numpy()

# Inisialisasi EKF
Q = np.eye(3) * 0.1  # Process noise covariance
R = np.eye(3) * 0.5  # Measurement noise covariance
ekf = ExtendedKalmanFilter(Q, R)

# Matriks A dan H
A = np.eye(3)  # State transition Jacobian
H = np.eye(3)  # Measurement Jacobian

# Estimasi EKF
estimates = []
try : 
    for z in gyro_measurements:
        z = z.reshape(3, 1)  # Bentuk ulang pengukuran menjadi vektor kolom
        ekf.predict(A)
        ekf.update(z, H)
        estimates.append(ekf.x.flatten())

    estimates = np.array(estimates)

    # Plot hasil
    time = data['time']  # Menggunakan timestamp dari file CSV
    kf_wx = []
    kf_wy = []
    kf_wz = []
    for i in range(len(estimates)) : 
        kf_wx.append(estimates[i, 0])
        kf_wy.append(estimates[i, 1])
        kf_wz.append(estimates[i, 2])

    print("Error Wx : ",err_wx:=np.mean(np.abs(data["wx"])))
    print("Error Filtered Wx : ",err_kf_ax:=np.mean(np.abs(estimates[:,0])))
    print("Error Wy : ",err_wy:=np.mean(np.abs(data["wy"])))
    print("Error Filtered Wy : ",err_kf_wy:=np.mean(np.abs(estimates[:,1])))
    print("Error Wz : ",err_wz:=np.mean(np.abs(data["wz"])))
    print("Error Filtered Wz : ",err_kf_wz:=np.mean(np.abs(estimates[:,2])))
    print("Wx :Error Filtered lebih kecil " if err_wx > err_kf_ax else "Wx: Error Filtered lebih besar")
    print("Wy: Error Filtered lebih kecil " if err_wy > err_kf_wy else "Wy: Error Filtered lebih besar")
    print("Wz: Error Filtered lebih kecil" if err_wz > err_kf_wz else "Wz: Error Filtered lebih besar")

    plt.figure(figsize=(12, 8))
    plt.subplot(3,1,1)
    plt.plot(time,1*(gyro_measurements[:,0]),label="Noisy Measurement X")
    plt.plot(time,1*(estimates[:,0]),label="Filtered Gyro-X")
    plt.xlabel('Time (s)')
    plt.ylabel('X Angular Velocity (m/s²)')
    plt.legend()
    plt.tight_layout()
    plt.subplot(3,1,2)
    plt.plot(time, 1*(gyro_measurements[:,1]),label="Noisy Measurement Y")
    plt.plot(time, 1*(estimates[:,1]),label="Filtered Gyro-Y")
    plt.xlabel('Time (s)')
    plt.ylabel('Y Angular Velocity (m/s²)')
    plt.legend()
    plt.tight_layout()
    plt.subplot(3,1,3)
    plt.plot(time, 1*(gyro_measurements[:,2]),label="Noisy Measurement Y")
    plt.plot(time, 1*(estimates[:,2]),label="Filtered Gyro-Z")
    plt.xlabel('Time (s)')
    plt.ylabel('Z Angular Velocity (m/s²)')
    plt.legend()
    plt.tight_layout()
    plt.show()
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
        "kf_ax" : data["kf_ax"],
        "kf_ay" : data["kf_ay"],
        "kf_az" : data["kf_az"],
        "kf_wx": kf_wx,
        "kf_wy": kf_wy,
        "kf_wz": kf_wz,
    }
    d = pd.DataFrame(dlogger)
    d.to_csv("data/data_imu/data_imu_ekf.csv",index=False)
except Exception as e:
    print("Error : ",str(e))
