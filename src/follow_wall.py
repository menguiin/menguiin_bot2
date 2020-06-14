#! /usr/bin/env python
# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
import math

pub_ = None
regions_ = {
    'right': 0,
    'fright': 0,
    'front': 0,
    'fleft': 0,
    'left': 0,
}
state_ = 0
state_dict_ = {
    0: 'find the wall',
    1: 'turn left',
    2: 'follow the wall',
    3: 'turn right'
}

def clbk_laser(msg):
    global regions_
    scan_filter = []

    for i in range(0, 359):
        scan_filter.append(msg.ranges[i])
        if scan_filter[i] == 0:
            scan_filter[i] = 3.5

    regions_ = {
        # 'right':  min(min(scan_filter[80:120]), 3.5),
        'fright': min(min(scan_filter[90:150]), 3.5),
        'front':  min(min(scan_filter[151:210]), 3.5),
        'fleft':  min(min(scan_filter[211:270]), 3.5),
        # 'left':   min(min(scan_filter[241:280]), 3.5),
    }

    take_action()

def change_state(state):
    global state_, state_dict_
    if state is not state_:
        print 'Wall follower - [%s] - %s' % (state, state_dict_[state])
        state_ = state

def take_action():
    global regions_
    regions = regions_
    msg = Twist()
    
    state_description = ''
    
    d = 0.25
    f = 0.3
    if regions['front'] > f and regions['fleft'] > f and regions['fright'] > d:
        state_description = 'case 1 - nothing'
        change_state(0)
    elif regions['front'] < f and regions['fleft'] > f and regions['fright'] > d:
        state_description = 'case 2 - front'
        change_state(1)
    elif regions['front'] > f and regions['fleft'] > f and regions['fright'] < d:
        state_description = 'case 3 - fright'
        change_state(2)
    elif regions['front'] > f and regions['fleft'] < f and regions['fright'] > d:
        state_description = 'case 4 - fleft'
        change_state(0)
    elif regions['front'] < f and regions['fleft'] > f and regions['fright'] < d:
        state_description = 'case 5 - front and fright'
        change_state(1)
    elif regions['front'] < f and regions['fleft'] < f and regions['fright'] > d:
        state_description = 'case 6 - front and fleft'
        change_state(0)
    elif regions['front'] < f and regions['fleft'] < f and regions['fright'] < d:
        state_description = 'case 7 - front and fleft and fright'
        change_state(1)
    elif regions['front'] > f and regions['fleft'] < f and regions['fright'] < d:
        state_description = 'case 8 - fleft and fright'
        change_state(2)
    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)

def find_wall():
    msg = Twist()
    msg.linear.x = 0.1
    msg.angular.z = -0.3
    return msg

def turn_left():
    msg = Twist()
    msg.angular.z = 0.3
    return msg

def turn_right():
    msg = Twist()
    msg.angular.z = -0.3
    return msg

def follow_the_wall():
    global regions_
    
    msg = Twist()
    msg.linear.x = 0.15
    return msg

def main():
    global pub_
    
    rospy.init_node('follow_wall')
    
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    rospy.Subscriber('/scan', LaserScan, clbk_laser)
    
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        msg = Twist()
        if state_ == 0:
            msg = find_wall()
        elif state_ == 1:
            msg = turn_left()
        elif state_ == 2:
            msg = follow_the_wall()
        elif state_ == 3:
            msg = turn_right()
            pass
        else:
            rospy.logerr('Unknown state!')
        
        pub_.publish(msg)
        
        rate.sleep()

if __name__ == '__main__':
    main()