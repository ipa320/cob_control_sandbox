#!/usr/bin/env python
import roslib; roslib.load_manifest("cob_mmcontroller")
import rospy
from std_msgs.msg import Float64
from pr2_controllers_msgs.msg import JointControllerState


def state_cb(data):
    print "StateCallback:\n"
    print data

def start_node():
    rospy.init_node('test_arm_velocity_controller', anonymous=True)
    #subscribe
    state_sub = rospy.Subscriber("/arm_2_velocity_controller/state", JointControllerState, state_cb)
    #advertise topic
    command_pub = rospy.Publisher('/arm_2_velocity_controller/command', Float64)
    rospy.sleep(1.0)
    
    msg = Float64(1.6)
    command_pub.publish(msg)
    while not rospy.is_shutdown():
        #command_pub.publish(msg)
        print "spin..."
        rospy.sleep(1.0)


if __name__ == '__main__':
    start_node()

