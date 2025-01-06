#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
class KalmanFilter:
    def __init__(self, Q, R):
        # State vector [a_x, b_x, a_y, b_y, a_z, b_z]
        self.x = np.zeros((6, 1))
        # Covariance matrix
        self.P = np.eye(6)
        # Process noise covariance
        self.Q = Q
        # Measurement noise covariance
        self.R = R

    def predict(self, A):
        # Predict state
        self.x = A @ self.x
        # Predict covariance
        self.P = A @ self.P @ A.T + self.Q

    def update(self, z, H):
        # Measurement residual
        y = z - H @ self.x
        # Measurement covariance
        S = H @ self.P @ H.T + self.R
        # Kalman Gain
        K = self.P @ H.T @ np.linalg.inv(S)
        # Update state
        self.x = self.x + K @ y
        # Update covariance
        self.P = (np.eye(6) - K @ H) @ self.P

# Simulasi data percepatan linear dengan bias dan noise
np.random.seed(42) 
data = pd.read_csv("data/data_imu/data_raw_imu_scene_2.csv")
true_acceleration = np.array([[0], [0], [0]])  # True acceleration (robot is stationary)
bias = np.array([[np.mean(data["ax"])], [np.mean(data["ay"])], [np.mean(data["az"])]])  # Bias pada masing-masing sumbu
# data['ax']-=bias[0]
# data['ay']-=bias[1]
# data['az']-=bias[2]
measurement_variance = 0.05  # Variance in accelerometer measurements
# accel_measurements = true_acceleration + bias + \
#     np.random.normal(0, measurement_variance, (60, 3, 1))  # Simulated noisy data
accel_measurements = data[['ax','ay','az']].to_numpy()

# Inisialisasi Kalman Filter
Q = np.eye(6) * 0.01  # Process noise covariance
R = np.eye(3) * measurement_variance  # Measurement noise covariance
kf = KalmanFilter(Q, R)

# Matriks A dan H
A = np.eye(6)  # State transition matrix
H = np.array([
    [1, 1, 0, 0, 0, 0],
    [0, 0, 1, 1, 0, 0],
    [0, 0, 0, 0, 1, 1]
])  # Measurement matrix

# Estimasi Kalman Filter
estimated_acceleration = []
estimated_bias = []
try : 
    for z in accel_measurements:
        kf.predict(A)
        kf.update(z, H)
        estimated_acceleration.append(kf.x[[0, 2, 4]].flatten())  # True acceleration estimates
        estimated_bias.append(kf.x[[1, 3, 5]].flatten())  # Bias estimates

    estimated_acceleration = np.array(estimated_acceleration)
    estimated_bias = np.array(estimated_bias)



    # Plot hasil estimasi percepatan
    time = data["time"]

    plt.figure(figsize=(12, 8))
    for i, axis in enumerate(['x', 'y', 'z']):
        plt.subplot(3, 1, i + 1)
        plt.plot(time, [z[i] for z in accel_measurements], label=f'Noisy Measurement {axis.upper()}', alpha=0.5)
        plt.plot(time, estimated_acceleration[:, i], label=f'Estimated Acceleration {axis.upper()}', color='green')
        plt.axhline(true_acceleration[i], color='red', linestyle='--', label='True Acceleration')
        plt.title(f'Estimation on {axis.upper()}-axis')
        plt.xlabel('Time (s)')
        plt.ylabel('Linear Acceleration (m/s²)')
        plt.legend()

    plt.tight_layout()
    # Plot hasil estimasi bias
    plt.figure(figsize=(12, 6))
    for i, axis in enumerate(['x', 'y', 'z']):
        plt.subplot(3, 1, i + 1)
        plt.plot(time, estimated_bias[:, i], label=f'Estimated Bias {axis.upper()}', color='blue')
        plt.axhline(bias[i], color='orange', linestyle='--', label='True Bias')
        plt.title(f'Bias Estimation on {axis.upper()}-axis')
        plt.xlabel('Time (s)')
        plt.ylabel('Bias (m/s²)')
        plt.legend()

    plt.tight_layout()
    plt.show()
    print(bias)

    dlogger = {
        "time": data["time"],
        "ax" : data["ax"],
        "ay" : data["ay"],
        "az" : data["az"],
        "or_x": data["or_x"],
        "or_y": data["or_y"],
        "or_z": data["or_z"],
        "or_w": data["or_w"],
        "estimated_ax" : estimated_acceleration[:, 0],
        "estimated_ay" : estimated_acceleration[:, 1],
        "estimated_az" : estimated_acceleration[:, 2],
        "bias_x" : [bias[0] for _ in range(len(data))],
        "bias_y" : [bias[1] for _ in range(len(data))],
        "bias_z" : [bias[2] for _ in range(len(data))],
        "estimated_bias_x" : estimated_bias[:, 0],
        "estimated_bias_y" : estimated_bias[:, 1],
        "estimated_bias_z" : estimated_bias[:, 2],
    }

    d = pd.DataFrame(dlogger)
    d.to_csv("data/data_imu/data_dynamic_bias_accel.csv",index=False)
except Exception as e:
    print("Error : " + str(e))
