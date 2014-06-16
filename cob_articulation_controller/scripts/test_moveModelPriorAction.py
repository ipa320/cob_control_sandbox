#! /usr/bin/env python

import roslib; roslib.load_manifest('cob_articulation_controller')
import rospy
import actionlib

import sys

from optparse import OptionParser

from cob_articulation_controller.msg import *

def main():
    parser = OptionParser()
    parser.add_option('-a', '--action_var', dest = 'action_var', default = 0.0,
                      action = 'store', help = "describes the movement relative to the start pose \
                      (e.g. opening angle in rad for a rotational articulation or distance in m \
                      for a prismatic one)")
    parser.add_option('-i', '--model_id', dest = 'model_id', default = -1,
                      action = 'store', help = "ID of model according to which movement is executed")
    parser.add_option('-d', '--database', dest = 'database', default = "",
                      action = 'store', help = "file name of database to load prior models from")
    parser.add_option('-t', '--target_duration', dest = 'duration', default = 20.0,
                      action = 'store', help = "duration in seconds until movement should be finished")
    (options, args) = parser.parse_args()

    rospy.init_node('moveModelPrior_client')
    client = actionlib.SimpleActionClient('move_model_prior', MoveModelPriorAction)
    client.wait_for_server()
    print "Server OK"

    # set up goal
    goal = MoveModelPriorGoal()
    if options.database == "":
        rospy.logerr("No database is given to load prior models from!")
        sys.exit("Database is missing! Use -d option")
    goal.database = options.database
    if options.model_id < 0:
        rospy.logerr("No model ID is given according to which a movement could be executed!")
        sys.exit("Model ID is missing! Use -i option")
    goal.model_id = int(options.model_id)
    goal.action_variable = float(options.action_var)
    goal.duration.secs = float(options.duration)

    client.send_goal(goal, feedback_cb=print_feedback)
    print client.get_state()
    while client.get_state() == 0 or client.get_state() == 1:
        client.wait_for_result(rospy.Duration.from_sec(1.0))
    print "RESULT:"
    print client.get_result()

def print_feedback(feedback):
    print "Feedback:"
    print feedback


if __name__ == '__main__':
    main()
