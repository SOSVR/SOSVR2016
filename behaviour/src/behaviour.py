#!/usr/bin/env python
"""
Behaviour, S.O.S.VR
Developed by: Sajjad Azami
June 2016
"""

import actionlib
import roslib
import rospy
import smach
import smach_ros
import turtlesim
import math
import tf
from tf import TransformListener
import random
from actionlib_msgs.msg import GoalStatusArray
from nav_msgs.msg import OccupancyGrid
from move_base_msgs.msg import *
from smach_ros import ServiceState

#####################################################
##################### VARIABLES #####################
goals_list = []
current_goal_status = ''

count = 0
goalstemp = []
for i in range(-10, 0):
    tmp = [i, i, 0, 1, 0, 0, 1]
    goalstemp.append(tmp)


################### END VARIABLES ###################
#####################################################

##################### FUNCTIONS #####################
#####################################################


# get current position of robot using tf translation
def get_current_position():
    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)
    flag = True
    trans = 0
    while flag and not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            rospy.loginfo(trans)
            flag = False
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
    return trans


# random goal generator
def get_random_goal():
    x = random.uniform(-2.0, 2.0)
    y = random.uniform(-2.0, 2.0)
    w = random.uniform(-2.0, 2.0)
    z = random.uniform(-2.0, 2.0)

    return [x, y, 0, w, 0, 0, z]


# subscriber method callback from /move_base/status
def callback_goal_status(data):
    current_goal_status = data.status_list[0].text


# subscriber method from /move_base/status
def listener_goal_status():
    rospy.init_node('goal_status_listener', anonymous=True)
    rospy.Subscriber("move_base/status", GoalStatusArray, callback_goal_status)
    rospy.spin()


# subscriber method callback from /move_base/global_costmap/costmap
def callback_global_costmap(self, data):
    global global_costmap
    global_costmap = data.data


# subscriber method from /move_base/global_costmap/costmap
def listener_global_costmap():
    # rospy.init_node('global_costmap_listener', anonymous=True)
    rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, callback_global_costmap)
    rospy.spin()


# publishes goal on move_base/goal using SimpleActionClient
# inputs: position x, y, z, orientation w, x, y, z
def move_to(pos_x, pos_y, pos_z, ornt_w, ornt_x, ornt_y, ornt_z):
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


################### END FUNCTIONS ###################
#####################################################

####################### STATES ######################
#####################################################


# define Init state
class Init(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['toExplore', 'toRescue'])

    def execute(self, userdata):
        rospy.loginfo('Executing Init')
        return 'toExplore'


# define Explore state
class Explore(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['victimSpotted', 'victimNotSpotted'])
        # rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, callback_global_costmap())

    def execute(self, userdata):
        # listener_global_costmap()
        rospy.loginfo('Executing state Explore')
        # goal_list_temp = [x, y, 0, w, 0, 0, x]  # TODO set right goals
        goal_temp = get_random_goal()  # get random goal
        goals_list.append(goal_temp)  # add goal to goal list(for further uses)
        current_position = get_current_position()  # current translation of robot array[2]
        move_to(goal_temp[0] + current_position[0], goal_temp[1] + current_position[1], goal_temp[2],
                goal_temp[3], goal_temp[4], goal_temp[5], goal_temp[6], )
        return 'victimNotSpotted'


# define InitExplore state (Inner State)
class InitExplore(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['canExplore', 'cannotExplore'])

    def execute(self, userdata):
        rospy.loginfo('Executing InitExplore')
        return 'canExplore'


# define InitRescue state (Inner State)
class InitRescue(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['canRescue', 'cannotRescue'])

    def execute(self, userdata):
        rospy.loginfo('Executing state 2')
        return 'canRescue'


# define Park state (Inner State)
class Park(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['parkSuccessful', 'parkFail'])

    def execute(self, userdata):
        rospy.loginfo('Executing state 2')
        return 'outcome2'


# define Shutdown state (Inner State)
class Shutdown(smach.State):
    def __init__(self):
        smach.State.__init__(self)

    def execute(self, userdata):
        rospy.loginfo('Shutdown')
        return 'outcome2'


# define PassTask state (Inner State)
class PassTask(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['canContinue', 'cannotContinue'])

    def execute(self, userdata):
        rospy.loginfo('PassTask')
        return 'outcome2'


#################### END STATES #####################
#####################################################


def main():
    rospy.init_node('behaviour')
    sm = smach.StateMachine(
        outcomes=['SHUTDOWN'])

    with sm:
        smach.StateMachine.add('INIT', Init(),
                               transitions={'toRescue': 'INIT_RESCUE', 'toExplore': 'INIT_EXPLORE'})

        smach.StateMachine.add('INIT_RESCUE', InitRescue(),
                               transitions={'canRescue': 'PARK', 'cannotRescue': 'PASS_TASK'})

        smach.StateMachine.add('INIT_EXPLORE', InitExplore(),
                               transitions={'canExplore': 'EXPLORE', 'cannotExplore': 'SHUTDOWN'})

        smach.StateMachine.add('EXPLORE', Explore(),
                               transitions={'victimSpotted': 'INIT_RESCUE', 'victimNotSpotted': 'INIT_EXPLORE'})

        smach.StateMachine.add('PARK', Park(),
                               transitions={'parkFail': 'PARK', 'parkSuccessful': 'SHUTDOWN'})

        smach.StateMachine.add('PASS_TASK', PassTask(),
                               transitions={'canContinue': 'INIT_EXPLORE', 'cannotContinue': 'SHUTDOWN'})

        sis = smach_ros.IntrospectionServer('Behavior', sm, '/SM_ROOT')
        sis.start()

        # Execute SMACH plan
        outcome = sm.execute()

        rospy.spin()
        sis.stop()


if __name__ == '__main__':
    main()
