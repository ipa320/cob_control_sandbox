#! /usr/bin/env python

import roslib; roslib.load_manifest('cob_articulation_controller')
import rospy
import actionlib

from cob_articulation_controller.msg import *

if __name__ == '__main__':
    rospy.init_node('openfridge_client')
    client = actionlib.SimpleActionClient('openFridgeDoor', OpenFridgeAction)
    client.wait_for_server()

    goal = OpenFridgeGoal()
    # Fill in the goal here
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(15.0))
