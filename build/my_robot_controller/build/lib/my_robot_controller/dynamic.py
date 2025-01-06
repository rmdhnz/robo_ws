#!/usr/bin/env python3
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

class EKFWithBiasCorrection:
    def __init__(self, dt, process_noise_cov, measurement_noise_cov, initial_state, initial_covariance):
        self.dt = dt  # Time step

        # State vector [p_x, p_y, p_z, v_x, v_y, v_z, roll, pitch, yaw, b_ax, b_ay, b_az, b_omega_x, b_omega_y, b_omega_z]
        self.state = np.array(initial_state)

        # Covariance matrix
        self.P = np.array(initial_covariance)

        # Process noise covariance
        self.Q = np.array(process_noise_cov)

        # Measurement noise covariance
        self.R = np.array(measurement_noise_cov)

        # Identity matrix
        self.I = np.eye(len(self.state))

    def predict(self, accel, gyro):
        """Predict step of EKF."""
        # State variables
        px, py, pz, vx, vy, vz, roll, pitch, yaw, bax, bay, baz, bwx, bwy, bwz = self.state

        # Corrected accelerations and gyro rates
        ax_corrected = accel[0] - bax
        ay_corrected = accel[1] - bay
        az_corrected = accel[2] - baz
        wx_corrected = gyro[0] - bwx
        wy_corrected = gyro[1] - bwy
        wz_corrected = gyro[2] - bwz

        # State prediction
        px += vx * self.dt
        py += vy * self.dt
        pz += vz * self.dt
        vx += ax_corrected * self.dt
        vy += ay_corrected * self.dt
        vz += az_corrected * self.dt
        roll += wx_corrected * self.dt
        pitch += wy_corrected * self.dt
        yaw += wz_corrected * self.dt

        self.state = np.array([px, py, pz, vx, vy, vz, roll, pitch, yaw, bax, bay, baz, bwx, bwy, bwz])

        # Jacobian of the state transition function
        F = np.eye(len(self.state))
        F[0, 3] = self.dt  # dp_x / dv_x
        F[1, 4] = self.dt  # dp_y / dv_y
        F[2, 5] = self.dt  # dp_z / dv_z
        F[3, 9] = -self.dt  # dv_x / db_ax
        F[4, 10] = -self.dt  # dv_y / db_ay
        F[5, 11] = -self.dt  # dv_z / db_az
        F[6, 12] = -self.dt  # droll / db_wx
        F[7, 13] = -self.dt  # dpitch / db_wy
        F[8, 14] = -self.dt  # dyaw / db_wz

        # Covariance prediction
        self.P = F @ self.P @ F.T + self.Q

    def update(self, measurement):
        """Update step of EKF."""
        # Measurement model
        H = np.zeros((6, 15))
        H[0, 9] = -1
        H[1, 10] = -1
        H[2, 11] = -1
        H[3, 12] = -1
        H[4, 13] = -1
        H[5, 14] = -1

        # Innovation
        z = np.array(measurement)  # Measurement [a_x, a_y, a_z, w_x, w_y, w_z]
        z_pred = H @ self.state  # Predicted measurement
        y = z - z_pred  # Residual

        # Innovation covariance
        S = H @ self.P @ H.T + self.R

        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)

        # State update
        self.state = self.state + K @ y

        # Covariance update
        self.P = (self.I - K @ H) @ self.P

    def get_state(self):
        """Get the current estimated state."""
        return self.state


# Main program
if __name__ == "__main__":
    dt = 0.01  # Time step

    # Initial state [p_x, p_y, p_z, v_x, v_y, v_z, roll, pitch, yaw, b_ax, b_ay, b_az, b_omega_x, b_omega_y, b_omega_z]
    initial_state = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0.1, 0.1, 0.1, 0.01, 0.01, 0.01]

    # Initial covariance
    initial_covariance = np.eye(15) * 0.1

    # Process noise covariance
    process_noise_cov = np.eye(15) * 0.01

    # Measurement noise covariance
    measurement_noise_cov = np.eye(6) * 0.05

    ekf = EKFWithBiasCorrection(dt, process_noise_cov, measurement_noise_cov, initial_state, initial_covariance)

    # Load IMU data from CSV
    imu_data = pd.read_csv("data/data_imu/data_raw_imu_scene_2.csv").values  # Convert to NumPy array

    # Store results for visualization
    positions = []
    orientations = []

    # Run EKF with data from CSV
    for i, row in enumerate(imu_data):
        accel = row[:3]  # [ax, ay, az]
        gyro = row[3:]   # [wx, wy, wz]
        ekf.predict(accel, gyro)
        ekf.update(row)

        # Store state
        state = ekf.get_state()
        positions.append(state[:3])  # [p_x, p_y, p_z]
        orientations.append(state[6:9])  # [roll, pitch, yaw]

    # Convert results to NumPy arrays
    positions = np.array(positions)
    orientations = np.array(orientations)

    # Plot results
    plt.figure(figsize=(12, 6))

    # Plot positions
    plt.subplot(2, 1, 1)
    plt.plot(positions[:, 0], label="p_x (Position X)")
    plt.plot(positions[:, 1], label="p_y (Position Y)")
    plt.plot(positions[:, 2], label="p_z (Position Z)")
    plt.title("Estimated Positions")
    plt.xlabel("Time Step")
    plt.ylabel("Position (m)")
    plt.legend()
    plt.grid()

    # Plot orientations
    plt.subplot(2, 1, 2)
    plt.plot(orientations[:, 0], label="roll (Orientation Roll)")
    plt.plot(orientations[:, 1], label="pitch (Orientation Pitch)")
    plt.plot(orientations[:, 2], label="yaw (Orientation Yaw)")
    plt.title("Estimated Orientations")
    plt.xlabel("Time Step")
    plt.ylabel("Orientation (rad)")
    plt.legend()
    plt.grid()

    # Show the plots
    plt.tight_layout()
    plt.show()
