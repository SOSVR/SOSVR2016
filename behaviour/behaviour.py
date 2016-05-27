#!/usr/bin/env python

import roslib

roslib.load_manifest('turtlesim')
import rospy
import smach
import turtlesim
import math
import smach_ros
from smach_ros import ServiceState
from turtlesim.srv import *


# define Init state
class Init(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['toExplore', 'toRescue'])

    def execute(self, userdata):
        rospy.loginfo('Executing state 1')
        return 'outcome1'


# define Explore state
class Explore(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['victimSpotted', 'victimNotSpotted'])

    def execute(self, userdata):
        rospy.loginfo('Executing state 1')
        return 'outcome1'


# define InitExplore state (Inner State)
class InitExplore(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['canExplore', 'cannotExplore'])

    def execute(self, userdata):
        rospy.loginfo('Executing state 1')
        return 'outcome1'


# define InitRescue state (Inner State)
class InitRescue(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['canRescue', 'cannotRescue'])

    def execute(self, userdata):
        rospy.loginfo('Executing state 2')
        return 'outcome2'


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
