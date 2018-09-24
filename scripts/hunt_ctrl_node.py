#!/usr/bin/env python

import random
import math
import time

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

# keep track of current hunter and runner poses
hunter_pose = Pose()
runner_pose = Pose()
move_msg = Twist()  # so we don't have to recreate every time


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


def _calc_rotation():
    """Calculates the needed rotation to turn hunter towards runner.

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
    target_theta = math.atan2(runner_pose.y - hunter_pose.y,
                              runner_pose.x - hunter_pose.x)
    # should we even do this?
    #if target_theta < 0.:
    #    target_theta += 2.*math.pi

    return target_theta - hunter_pose.theta


def _calc_distance():
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


def move_hunter_towards_runner(pub):
    """Rotates and moves the hunter in the direction of the runner.

    Parameters
    ----------
    pub : rospy.Publisher instance
        Publishes messages to the /hunter/cmd_vel topic.
    """
    global move_msg

    rotation = _calc_rotation()
    distance = _calc_distance()

    move_msg.linear.x = distance
    move_msg.angular.z = 20 * rotation

    pub.publish(move_msg)


def hunt_ctrl_node():
    """
    """
    pub = rospy.Publisher('/hunter/cmd_vel', Twist, queue_size=10)
    rospy.init_node('hunt_ctrl', anonymous=False)
    rospy.Subscriber('/hunter/pose', Pose, set_current_pose, 'hunter')
    rospy.Subscriber('/turtle1/pose', Pose, set_current_pose, 'runner')
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        move_hunter_towards_runner(pub)
        rate.sleep()

if __name__ == '__main__':
    try:
        time.sleep(1.)  # give other nodes time to initialize
        hunt_ctrl_node()
    except rospy.ROSInterruptException:
        pass
