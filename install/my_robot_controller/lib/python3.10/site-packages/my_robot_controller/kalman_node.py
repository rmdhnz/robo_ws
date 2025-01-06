#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

def mse_err(data,base=None) :
    if base is None : 
        base = np.array([0 for _ in range(len(data))])
    try : 
        return np.mean((data - base)**2)
    except Exception as e : 
        print(f"Error occurred: {str(e)}")
        return None
    


data = pd.read_csv("data/data_imu/data_imu_mechanization.csv")
# Parameter model
dt = 1.0  # Time step (seconds)
Q = 0.1  # Process noise covariance
R = 0.01   # Measurement noise covariance

# Non-linear state transition and measurement functions
def f(x):
    return x  # State transition model (x_k = x_{k-1})

def h(x):
    return x  # Measurement model (z_k = x_k)

# Jacobian of the state transition and measurement functions
def F(x):
    return np.array([[1]])  # Derivative of f with respect to x

def H(x):
    return np.array([[1]])  # Derivative of h with respect to x

# Initialization
x = np.array([[0]])  # Initial state (velocity in m/s)
P = np.array([[1]])  # Initial state covariance
num_steps = len(data["ax"])     # Number of time steps
waktu = data["time"]
ax = data["ax"]
yaw = data["yaw"]
ay = data["ay"]
bias_ax = np.mean(ax)
# Storage for estimated velocities
estimated_ax= np.zeros((len(waktu), 1))
# Extended Kalman Filter process
for k in range(num_steps):
    # Prediction
    x_pred = f(x)
    P_pred = F(x).dot(P).dot(F(x).T) + Q

    # Measurement
    z = yaw[k]

    # Kalman Gain
    K = P_pred.dot(H(x_pred).T).dot(np.linalg.inv(H(x_pred).dot(P_pred).dot(H(x_pred).T) + R))

    # Update
    x = x_pred + K.dot(z - h(x_pred))
    P = (np.eye(1) - K.dot(H(x_pred))).dot(P_pred)

    # Store the estimated velocity
    estimated_ax[k] = x
# Plotting results
yaw = np.array(yaw)

err_ekf = abs(estimated_ax)
err_gyro = abs(yaw)
print("Rata rata error EKF\t:",np.mean(err_ekf))
print("Rata rata error Gyro\t:",np.mean(err_gyro))
if __name__ == '__main__':
    plt.figure(1)
    plt.plot(waktu,err_gyro,label="Yaw")
    plt.plot(waktu,err_ekf,label="EKF")
    plt.xlabel("Time (s)")
    plt.ylabel("Error")
    plt.grid(True)
    plt.legend()
    plt.show()
