import numpy as np
from numpy import *
import pandas as pd
import matplotlib.pyplot as plt
from functions import forward_kinematics, Jacobian_2l2d, plots_2D, error_plots, peak_error_calc,plots_trackarm
import math

data_file = pd.read_csv(r'2 link planar.dat', sep=',', usecols=[1, 2, 3, 4])

theta_list = data_file['theta'].tolist()
rad_theta_list = np.deg2rad(theta_list)
print("thetas:", theta_list, rad_theta_list)
l = []
for i in range(0, len(theta_list)):
    l.append(data_file.iloc[i][2])
v = 60
velocity = [0, 0]
input_parameters = [200, 300, 0, 0.5, 0.01, v]
loop_control = 1
goal = [input_parameters[0], input_parameters[1], input_parameters[2]]
xc, yc, zc, _, _, CUM_POS = forward_kinematics(data_file, theta_list)
a = xc
b = yc
error = [0, 0, 0]
effector = [xc, yc, zc]
track_arm = []
error[0] = goal[0] - effector[0]
error[1] = goal[1] - effector[1]
error[2] = goal[2] - effector[2]
res_error = sqrt((error[0]**2) + (error[1]**2))
t = res_error / v
c = 0
iterations = []
time = [0]
final_pos = [[0], [0]]
velocities = []
while loop_control == 1:
    c += 1
    iterations.append(c)
    time.append(t+time[-1])
    xc, yc, zc, _, _, CUM_POS = forward_kinematics(data_file, theta_list)
    final_pos[0].append(xc)
    final_pos[1].append(yc)
    track_arm.append(CUM_POS)
    error = [0, 0, 0]
    effector = [xc, yc, zc]
    error[0] = goal[0] - effector[0]
    error[1] = goal[1] - effector[1]
    error[2] = goal[2] - effector[2]
    velocity[0] = (1/t) * error[0]
    velocity[1] = (1/t) * error[1]
    velocities.append(sqrt(velocity[0]**2 + velocity[1]**2))
    rad_theta_list = np.deg2rad(theta_list)
    if sqrt((error[0] ** 2) + (error[1] ** 2)) > input_parameters[4]:
        Jacobian = Jacobian_2l2d(rad_theta_list, l[0], l[1])
        theta_velocity = np.linalg.pinv(Jacobian) @ np.transpose(velocity)
        T = sqrt((error[0]**2) + (error[1]**2)) / sqrt(velocity[0]**2 + velocity[1]**2)
        del_t = T * input_parameters[3]
        for i in range(0, len(theta_list)):
            rad_theta_list[i] = rad_theta_list[i] + (del_t * theta_velocity[i])
            theta_list[i] = theta_list[i] + (del_t * np.rad2deg(theta_velocity[i]))
        print(theta_list)
        t = T
    else:
        loop_control = 0
        break

print("effector,c :", effector, c)
print("\n\n time:", time, len(time))
time.pop(0)
print("\n velocities", velocities)
