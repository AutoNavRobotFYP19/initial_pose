#!/usr/bin/env python
import rospy
import rospkg
import os

import math
import numpy as np
import matplotlib.pyplot as plt
from scipy import stats
from std_msgs.msg import String

time = []
x_map = []
x_gps = []
y_map = []
y_gps = []

if __name__ == '__main__':
    rospy.init_node('initial_pose')

    #open the robot_position file
    rospack = rospkg.RosPack()
    file_pkg = rospack.get_path('data_base')
    file_path = os.path.join(file_pkg, "robopose/robot_position.txt")

    if os.path.exists(file_path):
        with open(file_path, 'r') as file:
            for line in file:
                data = line.strip().split(",") 
                time.append(float(data[0]))
                x_map.append(float(data[1]))
                x_gps.append(float(data[5]))
                y_map.append(float(data[2]))
                y_gps.append(float(data[4]))
    else:
        print("File not found!")

    T = np.array(time)
    X_MAP = np.array(x_map)
    X_GPS = np.array(x_gps)
    Y_MAP = np.array(y_map)
    Y_GPS = np.array(y_gps)

    # get the data size
    n = len(X_MAP)
    lower = 0.2
    upper = 0.8
    start = int(n*lower)    # set upper and lower bounds (remove outliers)
    end = int(n*upper)

    # remove outliers from data
    T = T[start:end]
    X_MAP = X_MAP[start:end]
    X_GPS = X_GPS[start:end]
    Y_MAP = Y_MAP[start:end]
    Y_GPS = Y_GPS[start:end]

    print('array size:', n, ' start:', start, ' end:', end)

    # perform linear regression 
    # x-axis
    slope_x, intercept_x, r_x, p_x, std_err_x = stats.linregress(X_GPS, X_MAP)
    print("X axis")
    print('slope:', slope_x, 'intercept:', intercept_x, 'correlation:', r_x)
    # y-axis
    slope_y, intercept_y, r_y, p_y, std_err_y = stats.linregress(Y_GPS, Y_MAP)
    print("Y axis")
    print('slope:', slope_y, 'intercept:', intercept_y, 'correlation:', r_y)

    X_REG = slope_x*X_GPS + intercept_x
    Y_REG = slope_y*Y_GPS + intercept_y

    

