#!/usr/bin/env python

import random
import time

import rospy
from geometry_msgs.msg import Twist


def run_ctrl_node():
    """
    """
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.init_node('run_ctrl', anonymous=False)
    rate = rospy.Rate(10)   # continuous movement
    # will use this to change angular velocity every 2 seconds
    t0 = rospy.get_time()

    msg = Twist()
    msg.linear.x = 1
    msg.angular.z = random.uniform(-1, 1)

    while not rospy.is_shutdown():
        cur_time = rospy.get_time()
        if cur_time - t0 > 2.:
            msg.angular.z = random.uniform(-1, 1)
            t0 = cur_time
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        time.sleep(.5)  # give other nodes time to initialize
        run_ctrl_node()
    except rospy.ROSInterruptException:
        pass
