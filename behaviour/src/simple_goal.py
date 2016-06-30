#!/usr/bin/env python

import roslib
import rospy
import actionlib

# move_base_msgs
from move_base_msgs.msg import *


def move_to(pos_x, pos_y, pos_z, ornt_w, ornt_x, ornt_y, ornt_z):
    rospy.init_node('simple_move')

    # Simple Action Client
    sac = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    # create goal
    goal = MoveBaseGoal()

    # set goal
    goal.target_pose.pose.position.x = pos_x
    goal.target_pose.pose.position.y = pos_y
    goal.target_pose.pose.orientation.w = ornt_w
    goal.target_pose.pose.orientation.z = ornt_z
    goal.target_pose.header.frame_id = 'odom'
    goal.target_pose.header.stamp = rospy.Time.now()

    # start listener
    sac.wait_for_server()

    # send goal
    sac.send_goal(goal)

    # finish
    sac.wait_for_result()

    # print result
    print sac.get_result()


if __name__ == '__main__':
    try:
        move_to(4.5, 4.5, 0, 1, 0, 0, 1)
    except rospy.ROSInterruptException:
        print "Keyboard Interrupt"
