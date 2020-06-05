#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

vel = Twist()

def scan_callback(scan):
    rate = rospy.Rate(10)

    i = 0
    j = 0
    for i in range(80, 180):
        if scan.ranges[i] < 0.28 and scan.ranges[i] > 0:
            vel.angular.z = 0.3
        print("hi")
    for j in range(180, 280):
        if scan.ranges[j] < 0.28 and scan.ranges[j] > 0:
            vel.angular.z = 0.3
        print("no")
    print(vel.linear.x, vel.angular.z)
    run_pub.publish(vel)

    rate.sleep()


def run():
    global run_pub
    run_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rospy.init_node('vfh', anonymous=True)
    rospy.Subscriber('scan', LaserScan, scan_callback)
    vel.linear.x = 0.1

    rospy.spin()    

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass