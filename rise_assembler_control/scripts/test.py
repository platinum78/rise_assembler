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
from tf.transformations import quaternion_from_matrix
import numpy as np
    
if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("moveit_test_node")
    assembler = moveit_commander.RobotCommander()
    abb_irb120 = moveit_commander.MoveGroupCommander("abb_irb120")
    print abb_irb120.get_current_pose().pose