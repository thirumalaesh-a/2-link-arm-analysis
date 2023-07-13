import numpy as np
from numpy import *
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import math
##################################################################
from numpy.core._multiarray_umath import ndarray


def forward_kinematics(file, th):
    # theta = np.deg2rad(file['theta'].tolist())
    theta = np.deg2rad(th)
    alpha = np.deg2rad(file['alpha'].tolist())
    a = file['a'].tolist()
    d = file['d'].tolist()
    DoF = len(theta)

    # print("theta=",theta, "\n alpha = ", alpha)

    d_h_table_transpose = np.array([theta, alpha, a, d])
    print( "\n Transpose of DH table:", d_h_table_transpose)
    d_h_table = np.transpose(d_h_table_transpose)
    print("\n DH table is:", d_h_table, '\n \n')

    dh2ht_transform = []
    for i in range(0, DoF):
        dh2ht_transform.append(np.array([[np.cos(d_h_table[i, 0]), -np.sin(d_h_table[i, 0]) * np.cos(d_h_table[i, 1]),
                                          np.sin(d_h_table[i, 0]) * np.sin(d_h_table[i, 1]),
                                          d_h_table[i, 2] * np.cos(d_h_table[i, 0])],
                                         [np.sin(d_h_table[i, 0]), np.cos(d_h_table[i, 0]) * np.cos(d_h_table[i, 1]),
                                          -np.cos(d_h_table[i, 0]) * np.sin(d_h_table[i, 1]),
                                          d_h_table[i, 2] * np.sin(d_h_table[i, 0])],
                                         [0, np.sin(d_h_table[i, 1]), np.cos(d_h_table[i, 1]), d_h_table[i, 3]],
                                         [0, 0, 0, 1]]))
        print("table:", i, "::", dh2ht_transform)

    homogen_tran = dh2ht_transform[0]
    CUM_POS = [[0, dh2ht_transform[0][0][3]], [0, dh2ht_transform[0][1][3]], [0, dh2ht_transform[0][2][3]]]
    for i in range(0, DoF - 1):
        homogen_tran = homogen_tran @ dh2ht_transform[i + 1]
        CUM_POS[0].append(homogen_tran[0][3])
        CUM_POS[1].append(homogen_tran[1][3])
        CUM_POS[2].append(homogen_tran[2][3])

    print("The final transformed matrix:", homogen_tran)

    fx = homogen_tran[0][3]
    fy = homogen_tran[1][3]
    fz = homogen_tran[2][3]
    print("fx,fy,fz:", fx, fy, fz)

    return fx, fy, fz, homogen_tran, dh2ht_transform, CUM_POS


#################################################################
def plots_2D(CUM_POS, x_lim, y_lim, label_x, label_y):
    print("2D: plot's_x,plot's_y", CUM_POS[0], CUM_POS[1])

    plt.rcParams['legend.fontsize'] = 10
    fig = plt.figure()

    ax = fig.gca()
    plt.xlim(-x_lim, x_lim)
    plt.ylim(-y_lim, y_lim)
    ax.plot(CUM_POS[0], CUM_POS[1], 'ko-', label='curve')
    ax.plot(0, 0, 'ro-', label='zero')
    ax.legend()
    ax.set_xlabel(label_x)
    ax.set_ylabel(label_y)
    plt.show()

    return 0
######################################################################
def error_plots(x_values, y_values,x_errors, y_errors, z_errors):
    plt.rcParams['legend.fontsize'] = 10
    fig = plt.figure()

    ax = fig.gca()

    ax.plot(x_values, y_values, 'bo--', label='resultant error', linewidth=1)
    ax.plot(x_values, x_errors, 'ro--', label='x_errors')
    ax.plot(x_values, y_errors, 'yo--', label='y_errors')
    ax.plot(x_values, z_errors, 'go--', label='z_errors')
    ax.legend()
    ax.set_xlabel('iterations')
    ax.set_ylabel('errors')
    plt.show()

    return 0
########################################################
def Jacobian_2l2d(rad_theta_list,l1,l2):
    l3 = -l1
    l4 = -l2
    Jacobian = [
        [(l3 * math.sin(rad_theta_list[0])) - (l2 * math.sin((rad_theta_list[0] + rad_theta_list[1]))),
         l4 * math.sin((rad_theta_list[0] + rad_theta_list[1]))],
        [(l1 * math.cos(rad_theta_list[0])) + (l2 * math.cos((rad_theta_list[0] + rad_theta_list[1]))),
         l2 * math.cos((rad_theta_list[0] + rad_theta_list[1]))]
    ]
    print("Jacobian:", Jacobian)

    return Jacobian
#######################################################
def plots3D (CUM_POS, x_lim, y_lim, z_lim):
    # parametric curve

    plt.rcParams['legend.fontsize'] = 10

    fig = plt.figure()
    ax = fig.gca(projection='3d')
    plt.xlim(-x_lim, x_lim)
    plt.ylim(-y_lim, y_lim)
    ax.set_zlim(0, z_lim)
    ax.plot(0, 0, 0, 'ro-', label='parametric curve')
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    ax.plot(CUM_POS[0], CUM_POS[1], CUM_POS[2], 'bo-', label='parametric curve')
    ax.legend()

    plt.show()

    return 0
###################################################
def peak_error_calc(goal, pos):
    errors = []
    x1 = pos[0]
    y1 = pos[1]
    start = [x1[0], y1[0]]
    x1.pop(0)
    y1.pop(0)
    a = goal[1]-start[1]
    b = -goal[0]+start[0]
    c = (a*start[0]) + (b*start[1])      # ax+by+c = 0 is the equation of the desired track
    for i in range(0, len(x1)):
        errors.append(abs((a * x1[i] + b * y1[i] + c)) / (math.sqrt(a * a + b * b)))
    peak_error = max(errors)
    print("peak error:inside func:", peak_error)
    return peak_error

def plots_trackarm(CUM_POS,g, e):
    print("2D: plot's_x,plot's_y", CUM_POS[0], CUM_POS[1])

    plt.rcParams['legend.fontsize'] = 10
    fig = plt.figure()

    ax = fig.gca()
    plt.xlim(-420, 420)
    plt.ylim(-420, 420)
    for i in range(0, len(CUM_POS)):
    #    if i % 50 == 0:
        ax.plot(CUM_POS[i][0], CUM_POS[i][1], 'bo-', linewidth=0.3, label='curve')
    ax.plot([g[0], e[0]], [g[1], e[1]], 'yo--', linewidth=1, label='Home to Goal')
    ax.plot(0, 0, 'ro-', label='zero')
    ax.set_xlabel('pos_x')
    ax.set_ylabel('pos_y')
    plt.show()

    return 0

def plots_iterations_errors(xyy, x_lim):
    print("2D: plot's_x,plot's_y", xyy[0], xyy[1], xyy[2])
    fig, ax1 = plt.subplots()

    ax2 = ax1.twinx()
    ax1.plot(xyy[0], xyy[1], 'g--', marker='o')
    ax2.plot(xyy[0], xyy[2], 'b--', marker='o')

    ax1.set_xlabel('gain')
    ax1.set_ylabel('iterations', color='g')
    ax2.set_ylabel('peak error', color='b')

    plt.show()
    """
    plt.rcParams['legend.fontsize'] = 10
    fig = plt.figure()
    ax = fig.gca()
    plt.xlim(0, x_lim)
    plt.ylim(0, 1000)
    ax.plot(xyy[0], xyy[1], 'bo--', label='parametric curve')
    plt.xlim(0, x_lim)
    plt.ylim(0, 200)
    ax.plot(xyy[0], xyy[2], 'ro--', label='parametric curve')
    ax.plot(0, 0, 'ro--', label='zero')
    ax.legend()
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    plt.show()
    """

#############################################################################################

def manipulability(Jaco, center, CUM_POS):

    jaco_square_jjt = Jaco * np.transpose(Jaco)
    jaco_square_jtj = np.transpose(Jaco) @ Jaco
    inv_jjt = np.linalg.inv(jaco_square_jjt)
    inv_jtj = np.linalg.inv(jaco_square_jtj)
    print("inv", inv_jjt, "inv", inv_jtj)
    eigen_jjt_L, eigen_jjt_X = np.linalg.eig(inv_jjt)
    eigen_jtj_L, eigen_jtj_X = np.linalg.eig(inv_jtj)
    alpha = np.degrees(np.arctan2(eigen_jjt_X[1, 0], eigen_jjt_X[1, 1]))
    print("eigen_jjt", eigen_jjt_L, "and", eigen_jjt_X)
    print("\n eigen_jtj", eigen_jtj_L, "and", eigen_jtj_X)
    print("center", center)
    #r = np.sqrt(eigen_jjt_L)
    #r = sqrt(eigen_jjt_L[0]**2 + eigen_jjt_L[1]**2)
    #print("r", r)
    ells = []
    eigen_norm = eigen_jjt_L
    print(center, "center")
    if eigen_jjt_L[0] < 1e-6:
        eigen_norm[0] = 0.001
    if eigen_jjt_L[1] < 1e-6:
        eigen_norm[1] = 0.001
    center.pop()
    lower = -420
    upper = 420
    eigen_norm = [sqrt(abs(lower + (upper - lower) * x)) for x in eigen_jjt_L]
    # eigen_norm = [420*x/r for x in eigen_jjt_L]

    print("eigen_norm", eigen_norm)
    ells.append(Ellipse(center, eigen_norm[0], eigen_norm[1], angle=alpha))

    fig, ax = plt.subplots()#subplot_kw={'aspect': 'equal'})
   # plt.xlim(-500, 500)
   # plt.ylim(-500, 500)
    ax.set_xlim(-450, 450)
    ax.set_ylim(-450, 450)
    ax2 = ax.twinx()
    ax2.set_xlim(-450, 450)
    ax2.set_ylim(-450, 450)
    ax.plot(CUM_POS[0], CUM_POS[1], 'bo-', label='parametric curve')
    ax.plot(0, 0, 'ro--', label='zero')
    for e in ells:
        ax2.add_artist(e)
        e.set_clip_box(ax.bbox)
        e.set_alpha(0.5)
        e.set_facecolor('red')

    #plots_2D(CUM_POS, 420, 420)
    plt.show()


    return 0

def mani(j,e,c):
    jt = np.transpose(j)
    jjt = np.matmul(j, jt)
    print(j, jjt)
    #ijjt = np.linalg.pinv(jjt)
    l, x = np.linalg.eig(jjt)
    print("\n eig", l, "\n vec", x)
    alpha = np.degrees(np.arctan2(x[1, 0], x[1, 1]))
    print("aaaaaaa", x, "aaaaaaaa", x[1, 0], x[1, 1])
    print("l", l, "\n x", x, "\n a", alpha, "\ne-c", e)
    #l.sort()
    print("\nlll\n", l, "sqrt", sqrt(abs(l)))
    # noinspection PyTypeChecker
    ell = [Ellipse(xy=e, width=sqrt(l[0]), height=sqrt(l[1]), angle=alpha)]
    print("ell", ell)
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.set_xlim(-450, 450)
    ax.set_ylim(-450, 450)
    for e in ell:
        ax.add_artist(e)
        e.set_clip_box(ax.bbox)
        e.set_alpha(1)
        e.set_facecolor('red')
    ax.plot(c[0], c[1], 'bo-',alpha=0.5, label='robot')
    ax.plot(0, 0, 'ro--', label='zero')
    plt.show()