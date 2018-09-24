#!/usr/bin/env python

import random

import rospy
from turtlesim.srv import Spawn


def spawn_init_hunter():

    rospy.init_node('spawn_init_hunter', anonymous=False)
    rospy.wait_for_service('spawn')
    try:
        serv_func = rospy.ServiceProxy('spawn', Spawn)
        response = serv_func(x=random.uniform(0, 10),
                             y=random.uniform(0, 10),
                             theta=0,
                             name='hunter')
    except rospy.ServiceException as e:
        print "Service call failed: %s" % e

if __name__ == '__main__':
    spawn_init_hunter()
