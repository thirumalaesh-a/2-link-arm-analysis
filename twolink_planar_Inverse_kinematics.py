import numpy as np
from numpy import *
import pandas as pd
import matplotlib.pyplot as plt
from functions import forward_kinematics, Jacobian_2l2d, plots_2D, error_plots, peak_error_calc, plots_trackarm, \
    plots_iterations_errors
import math


##############################################################################
def IK_2d2l(input_parameters):
    data_file = pd.read_csv(r'2 link planar.dat', sep=',', usecols=[1, 2, 3, 4])
    theta_list = data_file['theta'].tolist()
    rad_theta_list = np.deg2rad(theta_list)
    print("thetas:", theta_list, rad_theta_list)

    loop_control = 1
    c = 0
    x_errors = []
    y_errors = []
    z_errors = []
    x_track = []
    y_track = []
    goal = [input_parameters[0], input_parameters[1], input_parameters[2]]
    print(goal)

    track_arm = []
    xc, yc, zc, _, _, CUM_POS = forward_kinematics(data_file, theta_list)
    a = xc
    b = yc
    length = []
    for i in range(0, len(theta_list)):
        length.append(data_file.iloc[i][2])

    while loop_control == 1:
        c += 1
        for i in range(0, len(theta_list)):
            if theta_list[i] > 360:
                while theta_list[i] > 360:
                    theta_list[i] = theta_list[i] - 360

        xc, yc, zc, _, _, CUM_POS = forward_kinematics(data_file, theta_list)

        x_track.append(xc)
        y_track.append(yc)
        track_arm.append(CUM_POS)
        # theta = np.deg2rad(data_file['theta'].tolist())
        # print("\n The cumulative positions:", CUM_POS)
        # rad_theta_list = np.deg2rad(theta_list)
        error = [0, 0, 0]
        effector = [xc, yc, zc]

        error[0] = goal[0] - effector[0]
        error[1] = goal[1] - effector[1]
        error[2] = goal[2] - effector[2]

        print("goal:", goal)
        print("effector:", effector)
        print("error:", error)
        print("c:", c)

        x_errors.append(error[0])
        y_errors.append(error[1])
        z_errors.append(error[2])
        if sqrt((error[0] ** 2) + (error[1] ** 2)) > input_parameters[4]:
            Jacobian = Jacobian_2l2d(rad_theta_list, length[0], length[1])  # THIS LINE CHANGES FOR DIFFERENT ROBOT CONFIGURATION
            del_effector = []

            for x in error:
                del_effector.append(x * input_parameters[3])
            if len(theta_list) == 2:
                del_effector = [del_effector[0], del_effector[1]]
            print(del_effector)
            print(np.linalg.pinv(Jacobian))
            del_theta = np.linalg.pinv(Jacobian) @ del_effector
            for i in range(0, len(theta_list)):
                rad_theta_list[i] = rad_theta_list[i] + del_theta[i]  #
                theta_list[i] = theta_list[i] + np.rad2deg(del_theta[i])  #
            print("theta:", theta_list)

            loop_control = 1

        else:
            loop_control = 0
            break

        print(theta_list)

    print("error:", error)
    print("x_errors:", x_errors, "\n y_errors:", y_errors)

    x_values = []
    y_values = []

    for i in range(0, len(x_errors)):
        x_values.append(i)
        y_values.append(sqrt(pow(x_errors[i], 2) + pow(y_errors[i], 2)))

    print("mean:", np.mean(y_values))
    print("std_dev:", np.std(y_values))
    print("range:", np.max(y_values), np.min(y_values))

    #error_plots(x_values, y_values, x_errors, y_errors, z_errors)

    #plots_2D(CUM_POS, 500, 500, "x_pos", "y_pos")
    # xx_track = x_track
    # yy_track = y_track
    # del xx_track[0::2]
    # del yy_track[0::2]
    # del xx_track[0::2]
    # del yy_track[0::2]
    track_xy = [x_track, y_track]
    #plots_2D(track_xy, 420, 420, "pos_x", "pos_y")
    goal.pop()
    #plots_trackarm(track_arm, goal, [a, b])
    peak_error_point = peak_error_calc(goal, track_xy)
    print("peak error:", peak_error_point)
    return c, peak_error_point, track_arm, Jacobian

    # for i in range(0,5):
    #    input_parameters[i]=float(input("enter goal_x,goal_y,goal_z,gain_beta,tolerance"))

#"""

no_of_iterations = []
peak_error_points = []
beta = []
bb = np.arange(0.01, 0.1, 0.01).tolist()
bb.append(0.1)
for b in bb:
    input_parameters = [-419, 0, 0, b, 1]
    x, y, t, j = IK_2d2l(input_parameters)
    # b += 0.1
    beta.append(b)
    no_of_iterations.append(x)
    peak_error_points.append(y)

# beta.pop()
plots_2D([beta, no_of_iterations], 1.2, max(no_of_iterations) + 50, "gain", "iterations")
plots_2D([beta, peak_error_points], 1.2, int(max(peak_error_points) + 10), "gain", "peak_errors")
plots_iterations_errors([beta, no_of_iterations, peak_error_points], 1.2)

"""
input_parameters = [0, 420, 0, 0.1, 0.01]
a, error, arm, ja = IK_2d2l(input_parameters)
print("iterations and peak error:", a,",", error)

#"""
def checkkk():
    return arm
