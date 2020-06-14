#!/usr/bin/env python

import rospy
import numpy as np 
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
from math import cos, sin

LINEAR_VEL = 0.22
STOP_DISTANCE = 0.2
LIDAR_ERROR = 0.05
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR

def scan_callback(scan):
    rate = rospy.Rate(0.5)
    lidar_polar = np.zeros((360, 2), dtype=float)
    lidar_xy = np.zeros((360, 2), dtype=float)
    for i in range(0, 359):
        lidar_polar[i, 0] = i
        lidar_polar[i, 1] = scan.ranges[i]
        if lidar_polar[i, 1] == 0:
            lidar_polar[i, 1] = 5
    for j in range(0, 359):
        lidar_xy[j, 0] = lidar_polar[j, 1] * cos(np.deg2rad(lidar_polar[j, 0]))
        lidar_xy[j, 1] = lidar_polar[j, 1] * sin(np.deg2rad(lidar_polar[j, 0]))
    lidar_xy = np.dot(lidar_xy, [[-1, 0], [0, -1]])
    plt.plot(lidar_xy[:, 0], lidar_xy[:, 1])
    plt.show()
    rate.sleep()

def main():
    rospy.init_node("polar_histogram", anonymous=True)
    rospy.Subscriber('scan', LaserScan, scan_callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass