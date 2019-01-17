#!/usr/bin/python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, sqrt
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import numpy as np
    
if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("moveit_test_node")
    assembler = moveit_commander.RobotCommander()
    abb_irb120 = moveit_commander.MoveGroupCommander("abb_irb120")
    pose = abb_irb120.get_current_pose().pose
    print pose
    print "Joint values: ", abb_irb120.get_current_joint_values()
    o = pose.orientation
    print "Euler orientation: ", euler_from_quaternion([o.x, o.y, o.z, o.w])