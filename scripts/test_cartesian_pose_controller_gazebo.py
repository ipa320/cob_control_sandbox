#!/usr/bin/env python
import roslib; roslib.load_manifest("cob_mmcontroller")
import rospy
from tf.transformations import *
from geometry_msgs.msg import PoseStamped, Twist


def gen_pose(frame_id="/odom_combined", pos=[0,0,0], euler=[0,0,0]):
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = pos
    pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = quaternion_from_euler(*euler)
    return pose

def error_cb(data):
    print "ErrorCallback:\n"
    print data

def pose_cb(data):
    print "PoseCallback:\n"
    print data

def start_node():
    rospy.init_node('test_cartesian_pose_controller', anonymous=True)
    #subscribe
    error_sub = rospy.Subscriber("/arm_cartesian_pose_controller/state/error", Twist, error_cb)
    pose_sub = rospy.Subscriber("/arm_cartesian_pose_controller/state/pose", PoseStamped, pose_cb)
    #advertise topic
    command_pub = rospy.Publisher('/arm_cartesian_pose_controller/command', PoseStamped)
    rospy.sleep(1.0)
    
    pose = gen_pose("/arm_0_link", pos=[-0.071, -0.023, 1.035], euler=[-0.001, 0.100, -2.845])
    command_pub.publish(pose)
    while not rospy.is_shutdown():
        #pose = gen_pose("arm_0_link", pos=[-0.071, -0.023, 1.035], euler=[-0.001, 0.100, -2.845])
        #command_pub.publish(pose)
        print "spin..."
        rospy.sleep(1.0)


if __name__ == '__main__':
    start_node()

