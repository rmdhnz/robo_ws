#!/usr/bin/env python3
import numpy as np
class SensorFusionWithSlip:
    def __init__(self, dt, process_noise, measurement_noise, initial_state, initial_covariance):
        self.dt = dt
        self.state = np.array(initial_state)  # [x, y, psi, vx, vy]
        self.P = np.array(initial_covariance)
        self.Q = np.array(process_noise)
        self.R = np.array(measurement_noise)
        self.I = np.eye(len(self.state))
        self.slip_threshold = 0.1  # Threshold to detect slip

    def predict(self, ax, ay, wz):
        x, y, psi, vx, vy = self.state
        x += vx * self.dt
        y += vy * self.dt
        psi += wz * self.dt
        vx += ax * self.dt
        vy += ay * self.dt
        self.state = np.array([x, y, psi, vx, vy])

        F = np.eye(len(self.state))
        F[0, 3] = self.dt
        F[1, 4] = self.dt
        self.P = F @ self.P @ F.T + self.Q

    def update(self, encoder, imu):
        # Calculate difference to detect slip
        v_enc = np.array([encoder[3], encoder[4]])
        v_imu = np.array([imu[3], imu[4]])
        delta_v = np.linalg.norm(v_enc - v_imu)

        if delta_v > self.slip_threshold:
            print("Slip detected! Using IMU for correction.")
            z = imu
        else:
            z = encoder

        H = np.eye(len(self.state))
        y = z - H @ self.state
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.state = self.state + K @ y
        self.P = (self.I - K @ H) @ self.P

    def get_state(self):
        return self.state


# Example Usage
dt = 0.1
initial_state = [0, 0, 0, 0, 0]
initial_covariance = np.eye(5) * 0.1
process_noise = np.eye(5) * 0.01
measurement_noise = np.eye(5) * 0.05

fusion = SensorFusionWithSlip(dt, process_noise, measurement_noise, initial_state, initial_covariance)

imu_data = [[0.2, 0.2, 0.01, 0.3, 0.2], [0.4, 0.1, 0.02, 0.4, 0.3], [0.6, 0.1, 0.01, 0.5, 0.4]]
encoder_data = [[0.2, 0.1, 0.02, 0.2, 0.1], [0.4, 0.3, 0.03, 0.3, 0.2], [0.6, 0.4, 0.04, 0.6, 0.3]]

for imu, encoder in zip(imu_data, encoder_data):
    fusion.predict(imu[0], imu[1], imu[2])
    fusion.update(encoder, imu)
    print("Estimated State:", fusion.get_state())
