#! /usr/bin/env python
# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
import math
from tf.transformations import euler_from_quaternion

pub_ = None
regions_ = {
    'right': 0,
    'front': 0,
    'left': 0,
}
state_ = 0
state_dict_ = {
    0: 'find the wall',
    1: 'turn left',
    2: 'turn right',
    3: 'follow the wall'
}

def clbk_laser(msg):
    global regions_
    r = rospy.Rate(100)
    scan_filter = []

    for i in range(0, 359):
        scan_filter.append(msg.ranges[i])
        if scan_filter[i] == 0:
            scan_filter[i] = 3.5

    regions_ = {
        'right': min(min(scan_filter[110:170]), 3.5),
        'front':  min(min(scan_filter[171:190]), 3.5),
        'left':  min(min(scan_filter[191:250]), 3.5)
    }

    take_action()
    r.sleep()   

def change_state(state):
    global state_, state_dict_
    if state is not state_:
        print 'Wall follower - [%s] - %s' % (state, state_dict_[state])
        state_ = state
    
def take_action():
    global regions_
    regions = regions_
    msg = Twist()
    rate = rospy.Rate(50)
    
    state_description = ''
    
    d = 0.3
    if regions['front'] > d and regions['left'] > d and regions['right'] > d:
        state_description = 'case 1 - nothing'
        change_state(0)
    elif regions['front'] < d and regions['left'] > d and regions['right'] > d:
        state_description = 'case 2 - front'
        change_state(1)
    elif regions['front'] > d and regions['left'] > d and regions['right'] < d:
        state_description = 'case 3 - right'
        change_state(3)
    elif regions['front'] > d and regions['left'] < d and regions['right'] > d:
        state_description = 'case 4 - left'
        change_state(0)
    elif regions['front'] < d and regions['left'] > d and regions['right'] < d:
        state_description = 'case 5 - front and right'
        change_state(1)
    elif regions['front'] < d and regions['left'] < d and regions['right'] > d:
        state_description = 'case 6 - front and left'
        change_state(1)
    elif regions['front'] < d and regions['left'] < d and regions['right'] < d:
        state_description = 'case 7 - front and left and right'
        change_state(1)
    elif regions['front'] > d and regions['left'] < d and regions['right'] < d:
        state_description = 'case 8 - left and right'
        change_state(3)
    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)
    
    rate.sleep()

def find_wall():
    msg = Twist()
    msg.linear.x = 0.1
    msg.angular.z = -0.5
    return msg

def turn_left():
    msg = Twist()
    msg.angular.z = 0.5

    return msg

def turn_right():
    msg = Twist()
    msg.angular.z = -0.5
    return msg

def follow_the_wall():
    msg = Twist()
    msg.linear.x = 0.1
    return msg

def main():   
    rospy.init_node('follow_wall')
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('/scan', LaserScan, clbk_laser)

    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        msg = Twist()
        if state_ == 0:
            msg = find_wall()
        elif state_ == 1:
            msg = turn_left()
        elif state_ == 2:
            msg = turn_right()
        elif state_ == 3:
            msg = follow_the_wall()
        
        else:
            rospy.logerr('Unknown state!')
        
        pub_.publish(msg)
        
        rate.sleep()

if __name__ == '__main__':
    main()