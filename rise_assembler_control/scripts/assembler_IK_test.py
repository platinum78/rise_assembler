#!/usr/bin/python2

import os, sys
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
print BASE_DIR
sys.path.append(BASE_DIR)

from assembler_controller import *
from assembler_tasks import *
import sys, copy, rospy, moveit_commander, moveit_msgs.msg, geometry_msgs.msg, time
from math import pi, sqrt
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import PositionIKRequest
from geometry_msgs.msg import Quaternion, Pose
from tf.transformations import quaternion_from_euler
import numpy as np

if __name__ == "__main__":
    # Initialize ROS and MoveIt!
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("moveit_test_node")
    assembler = moveit_commander.RobotCommander()
    abb_irb120 = moveit_commander.MoveGroupCommander("abb_irb120")

    rospy.wait_for_service("GetPositionIK")

    abb_irb120.clear_pose_targets()
    pose_target = Pose()
    pose_target.position.x = -0.059473
    pose_target.position.y = -0.273576
    pose_target.position.z = 0.233900
    
    q = quaternion_from_euler(0, pi/2, -pi/2)
    pose_target.orientation.x = q[0]
    pose_target.orientation.y = q[1]
    pose_target.orientation.z = q[2]
    pose_target.orientation.w = q[3]

    try:
        add_two_ints = rospy.ServiceProxy('GetPositionIK', AddTwoInts)
        resp1 = add_two_ints(x, y)
        return resp1.sum
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


    print joint_angle
    print dir(PositionIKRequest)