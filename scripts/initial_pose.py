#!/usr/bin/env
import rospy
import rospkg
import os

import math
import numpy as np
import matplotlib.pyplot as plt
from scipy import stats
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped

time = []
x_map = []
x_gps = []
y_map = []
y_gps = []

init_gps_x = []
init_gps_y = []

count = 0
state = 0

gps_count = 15

def cb_initial_gps(data):
    global state
    global count
    global gps_count

    if (count < gps_count):
        init_gps_x.append(data.point.x)
        init_gps_y.append(data.point.y)
        count += 1

    elif (count >= gps_count and state == 0):
        state = 1
        INIT_GPS_X = np.mean(init_gps_x)
        INIT_GPS_Y = np.mean(init_gps_y)

        INIT_MAP_X = slope_x*INIT_GPS_X + intercept_x
        INIT_MAP_Y = slope_y*INIT_GPS_Y + intercept_y

        init_pose_file_path = os.path.join(file_pkg, "robopose/initial.txt")

        if os.path.exists(init_pose_file_path):
            init_pose_file = open(init_pose_file_path, "r")
            line = init_pose_file.readline()
            pose_vals = line.split(" ") # get the imu yaw value at pose_vals[5]
            init_pose_file.close()
            
            init_pose_file = open(init_pose_file_path, "w")
            init_pose_file.write(str(INIT_MAP_X) + " " + str(INIT_MAP_Y) + " 0 0 0 " + pose_vals[5])
            print("\u001b[35mMap X:" + str(INIT_MAP_X) + "  Map Y:" +  str(INIT_MAP_Y) + "\u001b[37m")
            init_pose_file.close()

        else:
            print("File not found!")
     
    elif (state == 1):
        rospy.loginfo("\u001b[32m Completed Collecting Data \u001b[37m")
    


def listener():
    rospy.init_node("initial_pose", anonymous = True)
    rospy.Subscriber('/robot_rtk_xyz', PointStamped, cb_initial_gps)
    rospy.spin()    #run the node continunously until shutdown


if __name__ == '__main__':

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
                x_gps.append(float(data[4]))
                y_map.append(float(data[2]))
                y_gps.append(float(data[5]))
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

    # X_REG = slope_x*X_GPS + intercept_x
    # Y_REG = slope_y*Y_GPS + intercept_y

    try:
        listener()
    except rospy.ROSInterruptException:
        pass



    

