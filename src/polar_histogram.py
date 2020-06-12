#!/usr/bin/env python

import rospy
import numpy as np 
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
import math

LINEAR_VEL = 0.22
STOP_DISTANCE = 0.2
LIDAR_ERROR = 0.05
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR

def scan_callback(scan):
    rate = rospy.Rate(0.5)
    lidar = np.zeros((360, 2), dtype=float)

    for i in range(0, 359):
        lidar[i, 0] = i
        lidar[i, 1] = scan.ranges[i]
        if lidar[i, 1] == 0:
            lidar[i, 1] = 3.5

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