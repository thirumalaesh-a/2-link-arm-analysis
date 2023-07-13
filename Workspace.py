import numpy as np
from numpy import *
import pandas as pd
import matplotlib.pyplot as plt
from functions import forward_kinematics, Jacobian_2l2d, plots_2D, error_plots, peak_error_calc, plots_trackarm, manipulability, mani
import math
from matplotlib.patches import Ellipse


data_file = pd.read_csv(r'2 link planar.dat', sep=',', usecols=[1, 2, 3, 4])
ell = []
length = [210, 210]
#ii = [2, 90, 180, -90]
#jj = [2, 90]
for i in range(0, 360):
    for j in range(0, 360):
       if i % 20 == 0 and j % 30 == 0:
#for i in ii:
#    for j in jj:
            theta_list = [i, j]
            rad_theta_list = np.deg2rad(theta_list)
            centerx, centery, _, _, _, _ = forward_kinematics(data_file, theta_list)
            Jacobian = Jacobian_2l2d(rad_theta_list, length[0], length[1])
            j = Jacobian
            jt = np.transpose(j)
            jjt = np.matmul(j, jt)
            l, x = np.linalg.eig(jjt)
            alpha = np.degrees(np.arctan2(x[1, 0], x[1, 1]))
            ell.append(Ellipse(xy=[centerx, centery], width=sqrt(l[0]), height=sqrt(l[1]), angle=alpha))
            print("\n\nangles", theta_list)

fig = plt.figure()
ax = fig.add_subplot(111)
ax.set_xlim(-500, 500)
ax.set_ylim(-500, 500)
for e in ell:
    ax.add_artist(e)
    e.set_clip_box(ax.bbox)
    e.set_alpha(0.1)
    e.set_facecolor('red')
ax.plot(0, 0, 'ro--', label='zero')
plt.show()



