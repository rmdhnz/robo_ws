#!/usr/bin/env python3

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import tf_transformations


data = pd.read_csv("data/data_imu/data_imu_ekf.csv")
odom = pd.read_csv("data/data_odom/obstacle/data_odom_raw_scene_2.csv")
fusion = pd.read_csv("data/data_fusion/obstacle/ekf_fusion_data_scene_2.csv")

odom_wz = odom["wz"]
odom_wx = odom["wx"]
odom_wy = odom["wy"]
ax = data["ax"]
ay = data["ay"]
or_x = data["or_x"]
or_y = data["or_y"]
or_z = data["or_z"]
or_w = data["or_w"]
t = data["time"]
wx = data["wx"]
wy = data["wy"]
wz = data["wz"]
kf_wz = data["kf_wz"]
kf_ax = data["kf_ax"]
kf_ay = data["kf_ay"]
kf_x = [0]
kf_y = [0]
kf_vx,kf_vy = [0],[0]
bias_ax = -0.065
bias_ay = 0.0005
bias_wz = 0.00312
dt = 0.01
vx,vy = [0],[0]
yaw = [0]
kf_yaw = [0]
odom_yaw = odom["yaw"]
x,y = [0],[0]

for i in range(1,len(ax)) : 
    quaternion = np.array([
        or_x[i],
        or_y[i],
        or_z[i],
        or_w[i]
    ])
    linear_acc = np.array([
        ax[i],
        ay[i],
        0
    ])
    kf_linear_acc = np.array([
        kf_ax[i],
        kf_ay[i],
        0
    ])
    # wz[i]-=bias_wz
    # kf_wz[i]-=bias_wz

    rotation_matrix = R.from_quat(quaternion).as_matrix()
    acc_world = rotation_matrix.dot(linear_acc)
    kf_acc_world= rotation_matrix.dot(kf_linear_acc)
    vx.append(vx[i-1] + acc_world[0] * dt)
    kf_vx.append(kf_vx[i-1] + kf_acc_world[0]*dt)

    vy.append(vy[i-1] + acc_world[1] * dt)
    kf_vy.append(kf_vy[i-1] + kf_acc_world[1]*dt)

    x.append(x[i-1] + vx[i] * dt)
    kf_x.append(kf_x[i-1] + kf_vx[i]*dt)

    y.append(y[i-1] + vy[i] * dt)
    kf_y.append(kf_y[i-1] + kf_vy[i]*dt)

    # yaw.append(yaw[i-1] + wz[i] * dt)
    roll_angle,pitch_angle,yaw_angle = tf_transformations.euler_from_quaternion(quaternion)
    yaw.append(yaw_angle)
    kf_yaw.append(kf_yaw[i-1] + kf_wz[i] * dt)

# pi_num = [np.pi for _ in range(len(yaw))]
# for i in range(len(yaw)):
#     yaw[i] = (yaw[i] + pi_num[i]) % (2*np.pi) - pi_num[i]
#     kf_yaw[i] = (kf_yaw[i] + pi_num[i]) % (2*np.pi) - pi_num[i]

err_yaw = np.mean(np.abs(yaw))
err_kf_yaw = np.mean(np.abs(kf_yaw))
print("Mean error Yaw :",err_yaw)
print("Mean error kf_yaw :",err_kf_yaw)
result = "Error EKF yaw lebih kecil dengan selisih {}".format(err_yaw-err_kf_yaw) if err_kf_yaw < err_yaw else "Error EKF yaw lebih besar"
print("Result :", result)


plt.figure(figsize=(12,8))
# plt.subplot(2,2,1)
plt.title(" yaw IMU")
plt.plot(t,yaw,label="yaw mechanization")
# plt.plot(t,kf_yaw,label="yaw Kalman Filter")
# plt.plot(t,fusion["yaw"],label="Yaw Fusion")
plt.xlabel("Time (seconds)")
plt.grid(True)
plt.ylabel("Rad")
plt.legend()
plt.tight_layout()
# plt.show()
# exit()

plt.figure(figsize=(12,8))
# plt.subplot(2,2,2)
plt.title("Position-X ")
plt.plot(t,np.abs(x),label="Position-X")
plt.plot(t,np.abs(kf_x),label="Position-X-Filtered")
# plt.plot(t,fusion["x"],label="Fusion",linestyle="dashed",linewidth="3",color="blue")
# plt.plot(t,odom["x"],label="Odometry",color="magenta")
plt.xlabel("Time (s)")
plt.ylabel("Position X (m)")
plt.grid(True)
plt.legend()

plt.figure(figsize=(12,8))
# plt.subplot(2,2,3)
plt.title("Position-Y ")
plt.plot(t,y,label="Position-Y")
plt.plot(t,kf_y,label="Position-Y-Filtered")
# plt.plot(t,fusion["y"],label="Fusion",linestyle="dashed",linewidth="4",color="blue")
# plt.plot(t,odom["y"],label="Odometry",color="magenta")
plt.xlabel("Time (s)")
plt.ylabel("Position-Y (m)")
plt.grid(True)
plt.legend()
plt.figure(figsize=(12,8))
# plt.subplot(2,2,4)
plt.title("Position-Robot")
plt.plot(x,y,label="X-Y-position")
plt.plot(kf_x,kf_y,label="Filtered Mechanization")
# plt.plot(odom["x"],odom["y"],label="Odometry",color="magenta")
# plt.plot(fusion["x"],fusion["y"],label="Fusion",color="blue",linewidth="3",linestyle="dashed")
plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.grid(True)
plt.legend()
plt.tight_layout()

plt.show()
# print("Rata - rata error yaw:",np.mean(err_yaw))
# print("Rata - rata error EKF yaw:",np.mean(err_kf_yaw))
# print("Rata - rata error X:",np.mean(err_x))
# print("Rata - rata error EKF X:",np.mean(err_kf_x))
# print("Rata - rata error y:",np.mean(err_y))
# print("Rata - rata error EKF y:",np.mean(err_kf_y))
dlogger = {
    "time" : data["time"],
    "ax" : ax,
    "ay" : ay,
    "az" : data["az"],
    "wx" : data["wx"],
    "wy" : data["wy"],
    "wz" : data["wz"],
    "or_x" : data["or_x"],
    "or_y" : data["or_y"],
    "or_z" : data["or_z"],
    "or_w" : data["or_w"],
    "x" : x,
    "y" : y,
    "yaw" : yaw,
    "kf_yaw" : kf_yaw,
}
kf_yaw=np.array(kf_yaw)
yaw = np.array(yaw)
print("MSE :",np.mean((yaw-kf_yaw)**2))
d = pd.DataFrame(dlogger)
d.to_csv("data/data_imu/data_imu_mechanization.csv",index=False) 