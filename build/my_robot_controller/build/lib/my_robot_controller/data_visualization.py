#!/usr/bin/env python3
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
def mse_err(data,base=None) :
    if base is None : 
        base = np.array([0 for _ in range(len(data))])
    try : 
        return np.mean((data - base)**2)
    except Exception as e : 
        print(f"Error occurred: {str(e)}")
        return None
data = pd.read_csv("data/data_imu/data_raw_imu.csv")
ax = data["ax"]
ay = data["ay"]
az = data["az"]
t = data["time"]
wx = data["wx"]
wy = data["wy"]
wz = data["wz"]
az_val = az - 9.8
bias_ax = np.mean(ax)
bias_ay = np.mean(ay)
bias_az = np.mean(az_val)
bias_wz = np.mean(wz)
bias_wx = np.mean(wx)
bias_wy = np.mean(wy)
#print semua bias
print("Bias Ax:", bias_ax)
print("Bias Ay:", bias_ay)
print("Bias Az:", bias_az)
print("Bias Wx:", bias_wx)
print("Bias Wy:", bias_wy)
print("Bias Wz:", bias_wz)



ax-=bias_ax
ay-=bias_ay
az-=bias_az
wz-=bias_wz
wx-=bias_wx
wy-=bias_wy



plt.figure(figsize=(12,8))
plt.subplot(3,1,1)
plt.plot(t,ax,label="Corrected Measured")
plt.axhline(bias_ax, color='red', linestyle='--', label='Bias')
plt.xlabel("Time (s)")
plt.ylabel("Ax (m/s^2)")
plt.title("Acceleration-X")
plt.legend()
plt.grid(True)

plt.subplot(3,1,2)
plt.plot(t,ay,label="Corrected Measured")
plt.axhline(bias_ay, color='red', linestyle='--', label='Bias')
plt.xlabel("Time (s)")
plt.ylabel("Ay (m/s^2)")
plt.title("Acceleration-Y")
plt.legend()
plt.grid(True)

plt.subplot(3,1,3)
plt.plot(t,az,label="Corrected Measured")
plt.axhline(bias_az, color='red', linestyle='--', label='Bias')
plt.xlabel("Time (s)")
plt.ylabel("Az (m/s^2)")
plt.title("Acceleration-Z")
plt.legend()
plt.grid(True)
plt.tight_layout()

plt.figure(figsize=(12,8))
plt.subplot(3,1,1)
plt.plot(t,wx,label="Corrected Measured")
plt.axhline(bias_wx, color='red', linestyle='--', label='Bias')
plt.xlabel("Time (s)")
plt.ylabel("Wx (rad/s)")
plt.title("Angular Velocity-X")
plt.legend()
plt.legend()
plt.grid(True)

plt.subplot(3,1,2)
plt.plot(t,wy,label="Corrected Measured")
plt.axhline(bias_wy, color='red', linestyle='--', label='Bias')
plt.xlabel("Time (s)")
plt.ylabel("wy (rad/s)")
plt.title("Angular Velocity-Y")
plt.legend()
plt.grid(True)

plt.subplot(3,1,3)
plt.plot(t,wz,label="Corrected Measured")
plt.axhline(bias_wz, color='red', linestyle='--', label='Bias')
plt.xlabel("Time (s)")
plt.ylabel("wz (rad/s)")
plt.title("Angular Velocity-Z")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
