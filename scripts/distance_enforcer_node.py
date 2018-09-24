#!/usr/bin/env python

import random
import time

import rospy
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, Kill
from std_srvs.srv import Empty

# keep track of current hunter and runner poses
hunter_pose = Pose()
runner_pose = Pose()


def set_current_pose(data, key):
    """Updates the pose parameters on either the runner or hunter.

    Parameters
    ----------
    data : turtlesim.msg.Pose instance
        Current pose.
    key : str, choices: {'runner', 'hunter'}
    """
    mapping = {
        'hunter': hunter_pose,
        'runner': runner_pose,
    }

    mapping[key].x = data.x
    mapping[key].y = data.y
    mapping[key].theta = data.theta


def _calc_dist():
    """Calculates the distance between two poses.

    Uses the following global variables
    -----------------------------------
    hunter_pose : turtlesim.msg.Pose instance
        Pose of hunter.
    runner_pose : turtlesim.msg.Pose instance
        Pose of runner

    Returns
    -------
    float
    """
    return (((hunter_pose.x - runner_pose.x)**2
            + (hunter_pose.y - runner_pose.y)**2)**0.5)


def distance_enforcer():
    """Calculates distance between runner and hunter.
    """
    global runner_pose
    runner_pose.x = 100
    runner_pose.y = 100

    rospy.init_node('distance_enforcer', anonymous=False)
    rospy.Subscriber('/turtle1/pose', Pose, set_current_pose, 'runner')
    rospy.Subscriber('/hunter/pose', Pose, set_current_pose, 'hunter')
    kill_func = rospy.ServiceProxy('kill', Kill)
    clear_func = rospy.ServiceProxy('clear', Empty)
    spawn_func = rospy.ServiceProxy('spawn', Spawn)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        distance_apart = _calc_dist()
        print '%f' % distance_apart
        if distance_apart < 1.:
            # kill runner
            rospy.wait_for_service('kill')
            try:
                _ = kill_func(name='turtle1')
            except rospy.ServiceException as e:
                print "Service call failed: %s" % e

            # clear tracks
            rospy.wait_for_service('clear')
            try:
                _ = clear_func()
            except rospy.ServiceException as e:
                print "Service call failed: %s" % e

            # create a new runner at a random location
            rospy.wait_for_service('spawn')
            try:
                _ = spawn_func(x=random.uniform(0, 10),
                               y=random.uniform(0, 10),
                               theta=0, name='turtle1')
                # resets distance to high values
                runner_pose.x = 100
                runner_pose.y = 100
            except rospy.ServiceException as e:
                print "Service call failed: %s" % e

        rate.sleep()


if __name__ == '__main__':
    try:
        time.sleep(1.)
        distance_enforcer()
    except rospy.ROSInterruptException:
        pass
