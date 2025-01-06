#!/usr/bin/env python3

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
data = pd.read_csv("data/data_imu/data_dynamic_bias_accel.csv")
ax = data["ax"]
ay = data["ay"]
or_x = data["or_x"]
or_y = data["or_y"]
or_z = data["or_z"]
or_w = data["or_w"]
est_ax = data["estimated_ax"]
est_ay = data["estimated_ay"]
bias_x = data["estimated_bias_x"]
bias_y = data["estimated_bias_y"]
t = data["time"]
vx,vy = [0],[0]
x,y = [0],[0]
dt = 0.01

for i in range(1,len(ax)) : 
    quaternion = np.array([
        or_x[i],
        or_y[i],
        or_z[i],
        or_w[i]
    ])
    linear_acc = np.array([
        ax[i]-bias_x[i],
        ay[i]-bias_y[i],
        0
    ])

    rotation_matrix = R.from_quat(quaternion).as_matrix()
    acc_world = rotation_matrix.dot(linear_acc)
    vx.append(vx[i-1] + acc_world[0] * dt)
    vy.append(vy[i-1] + acc_world[1] * dt)

    x.append(x[i-1] + vx[i] * dt)
    y.append(y[i-1] + vy[i] * dt)


plt.figure(1)
plt.plot(x,y)
plt.show()