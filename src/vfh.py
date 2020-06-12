#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from math import radians, copysign, sqrt, pow, pi, atan2, cos
from tf.transformations import euler_from_quaternion
import numpy as np

LINEAR_VEL = 0.22
STOP_DISTANCE = 0.2
LIDAR_ERROR = 0.05
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR

class VFH():
    def __init__(self):
        rospy.init_node("vector_field", anonymous=False)
        rospy.on_shutdown(self.shutdown)
        self.vel = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.scan = rospy.Subscriber("scan", LaserScan, get_scan)
        self.tf_listener = tf.TransformListener()
        self.odom_frame = 'odom'
        move = Twist()
        pose = Point()
        r = rospy.Rate(10)
        
        try:
            self.tf_listener.waitForTransform(self.odom_frame, 'base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = 'base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, 'base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = 'base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between odom and base_link or base_footprint")
                rospy.signal_shutdown("tf Exception")

        


if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            VFH()
    except:
        rospy.loginfo("shutdown program.")