#!/usr/bin/env python
import roslib; roslib.load_manifest('cob_mmcontroller')

import rospy
from pr2_mechanism_msgs.srv import *
from pr2_mechanism_msgs.msg import *
from pr2_controllers_msgs.msg import JointControllerState
from std_msgs.msg import Float64


class init_torso:
    def __init__(self):
        rospy.wait_for_service('/pr2_controller_manager/switch_controller')

        #rospy.Subscriber("/torso_lower_neck_tilt_velocity_controller/state", JointControllerState, self.lower_neck_tilt_state_callback)
        #rospy.Subscriber("/torso_pan_velocity_controller/state", JointControllerState, self.pan_state_callback)
        rospy.Subscriber("/torso_upper_neck_tilt_velocity_controller/state", JointControllerState, self.upper_neck_tilt_state_callback)

        #self.lower_neck_tilt_pub = rospy.Publisher('/torso_lower_neck_tilt_velocity_controller/command', Float64)
        #self.pan_pub = rospy.Publisher('/torso_pan_velocity_controller/command', Float64)
        self.upper_neck_tilt_pub = rospy.Publisher('/torso_upper_neck_tilt_velocity_controller/command', Float64)
        
        
        try:
            switch_client = rospy.ServiceProxy('/pr2_controller_manager/switch_controller', SwitchController)
            
            req = SwitchControllerRequest()
            req.start_controllers.append("torso_lower_neck_tilt_velocity_controller")
            req.start_controllers.append("torso_pan_velocity_controller")
            req.start_controllers.append("torso_upper_neck_tilt_velocity_controller")
            req.stop_controllers.append("torso_controller")
            req.strictness = 2
            
            #res = switch_client(req)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        
        #msg = Float64(0.2)
        #self.lower_neck_tilt_pub.publish(msg)
        #self.pan_pub.publish(msg)
        #self.upper_neck_tilt_pub.publish(msg)
        #rospy.sleep(0.1)
    
    
    def lower_neck_tilt_state_callback(self, data):
        rospy.loginfo("LOWER: set_point: %f \t process_value: %f \t error: %f \t command: %f" %(data.set_point, data.process_value, data.error, data.command))
        msg = Float64(0.0)
        #msg = Float64(-data.process_value)
        self.lower_neck_tilt_pub.publish(msg)
        
    def pan_state_callback(self, data):
        rospy.loginfo("PAN: set_point: %f \t process_value: %f \t error: %f \t command: %f" %(data.set_point, data.process_value, data.error, data.command))
        msg = Float64(0.0)
        #msg = Float64(-data.process_value)
        self.pan_pub.publish(msg)
        
    def upper_neck_tilt_state_callback(self, data):
        rospy.loginfo("UPPER: set_point: %f \t process_value: %f \t error: %f \t command: %f" %(data.set_point, data.process_value, data.error, data.command))
        msg = Float64(0.0)
        #msg = Float64(-data.process_value)
        self.upper_neck_tilt_pub.publish(msg)


def main():
    try:
        rospy.init_node('cob_init_torso_velocity_controller')
        init_torso()
        rospy.spin()
    except rospy.ROSInterruptException: pass

if __name__ == '__main__':
    main()
