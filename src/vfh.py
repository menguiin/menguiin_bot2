#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry
from math import radians, copysign, sqrt, pow, pi, atan2, cos, sin
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
        move = Twist()
        position = Point()
        r = rospy.Rate(10)
        
            
        (position, rotation, velocity) = self.get_odom()
        last_rotation = 0
        linear_speed = 1
        angular_speed = 1
        (goal_x, goal_y, goal_z) = [20.0, 0.0, 0.0]
        goal_distance = sqrt(pow(goal_x - position.x, 2) + pow(goal_y - position.y, 2))
        distance = goal_distance
        w = 0.8 # damping weight

        while distance > LIDAR_ERROR:
            (position, rotation, velocity) = self.get_odom()
            x_start = position.x
            y_start = position.y
            lidar_xy, distance_obs = self.get_scan()
            repulsive_force_ = np.zeros((360, 2), dtype=float)
            attracting_force = []
            # repulsive
            for i in range(90, 270):
                repulsive_force_[i] = [(x_start - lidar_xy[i, 0])/distance_obs[i], (y_start - lidar_xy[i, 1])/distance_obs[i]]
            repulsive_force = np.sum(repulsive_force_, axis=0)
            vector_size = np.linalg.norm([velocity[0], velocity[1]])*np.linalg.norm(repulsive_force)
            if abs(vector_size) < 0.05:
                cosine = 0
            else:
                cosine = (velocity[0]*repulsive_force[0] + velocity[1]*repulsive_force[1])/(np.linalg.norm([velocity[0], velocity[1]])*np.linalg.norm(repulsive_force))
           
            damping = w*repulsive_force + (1-w)*repulsive_force*(-cosine)
            # attracting
            attracting_force = [(goal_x-position.x)/goal_distance, (goal_y-position.y)/goal_distance]
            
            # resultant
            R = damping + attracting_force  
            delta = atan2(R[1]/np.linalg.norm(R), R[0]/np.linalg.norm(R))
            

            move.linear.x = min(np.linalg.norm(R)*(1-cosine)*0.5, LINEAR_VEL)
            move.angular.z = 5*delta
            print([move.linear.x, move.angular.z])
            last_rotation = rotation
            self.vel.publish(move)
            r.sleep()


    def get_scan(self):
        scan = rospy.wait_for_message('scan', LaserScan)
        r = rospy.Rate(10)
        lidar_polar = np.zeros((360, 2), dtype=float)
        lidar_xy = np.zeros((360, 2), dtype=float)
        distance = np.zeros((360), dtype=float)
        for i in range(0, 359):
            lidar_polar[i, 0] = i
            lidar_polar[i, 1] = scan.ranges[i]
            if lidar_polar[i, 1] == 0:
                lidar_polar[i, 1] = 5
        for j in range(0, 359):
            lidar_xy[j, 0] = lidar_polar[j, 1] * cos(np.deg2rad(lidar_polar[j, 0]))
            lidar_xy[j, 1] = lidar_polar[j, 1] * sin(np.deg2rad(lidar_polar[j, 0]))
            distance[j] = np.linalg.norm(lidar_xy[j, :])
        lidar_xy = np.dot(lidar_xy, [[-1, 0], [0, -1]])
        
        r.sleep()
        return lidar_xy, distance

    def get_odom(self):
        r = rospy.Rate(10)
        odom = rospy.wait_for_message('odom', Odometry)
        position = [odom.pose.pose.position.x, odom.pose.pose.position.y, 0]
        rotation = euler_from_quaternion([odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w]) 
        velocity = [odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.angular.z]

        r.sleep()
        return (Point(*position), rotation[2], velocity)

    def shutdown(self):
        self.vel.publish(Twist())
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            VFH()
    except:
        rospy.loginfo("shutdown program.")
