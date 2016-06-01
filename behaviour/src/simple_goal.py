#!/usr/bin/env python

import roslib
import rospy
import actionlib

#move_base_msgs
from move_base_msgs.msg import *

def simple_move():

    rospy.init_node('simple_move')

    #Simple Action Client
    sac = actionlib.SimpleActionClient('move_base', MoveBaseAction )
    #sac2 = actionlib.SimpleActionClient('move_base_simple', MoveBaseAction )

    #create goal
    goal = MoveBaseGoal()

    #use self?
    #set goal
    goal.target_pose.pose.position.x = 10.0
    goal.target_pose.pose.position.y = 10.0
    goal.target_pose.pose.orientation.w = 0.8
    goal.target_pose.pose.orientation.z = 0.8
    goal.target_pose.header.frame_id = 'odom
    '
    goal.target_pose.header.stamp = rospy.Time.now()

    #start listner
    sac.wait_for_server()

    #send goal
    sac.send_goal(goal)

    #finish
    sac.wait_for_result()

    #print result
    print sac.get_result()


if __name__ == '__main__':
    try:
        simple_move()
    except rospy.ROSInterruptException:
        print "Keyboard Interrupt"
