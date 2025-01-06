#!/usr/bin/env python3

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

data = pd.read_csv("data/data_lidar/obstacle/data_raw_lidar_scene_2.csv")
data_odom = pd.read_csv("data/data_odom/data_odom_raw_diam.csv")

plt.plot(data["x"],data["y"])
plt.show()

exit()

mse_x = np.mean((data_odom["x"])**2 - 0)
mse_y = np.mean((data_odom["y"])**2 - 0)
print("MSE x : ", mse_x)
print("MSE y : ", mse_y)
plt.figure(figsize=(12,8))
plt.subplot(2,1,1)
plt.title("Error X Position Odometry")
plt.xlabel("Time (s)")
plt.ylabel('m')
plt.plot(data_odom["x"])
plt.subplot(2,1,2)
plt.title("Error X Position Odometry")
plt.plot(data_odom["y"])
plt.xlabel("Time (s)")
plt.ylabel('m')
plt.tight_layout()
plt.show()