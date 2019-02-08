#!/usr/bin/python2

import os, sys
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
print BASE_DIR
sys.path.append(BASE_DIR)

from assembler_controller import *
from assembler_tasks import *
from points_setup import *

import sys, copy, rospy, moveit_commander, moveit_msgs.msg, geometry_msgs.msg, time
from math import pi, sqrt
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler
import numpy as np

if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("moveit_test_node")

    assembler_cmd = moveit_commander.RobotCommander()
    abb_irb120 = moveit_commander.MoveGroupCommander("abb_irb120")
    assembler = rise_assembler_controller(assembler_cmd, abb_irb120)
    assembler.move_to_init_pos()
    rospy.loginfo("SYSTEM READY!")

    r2d = 180 / pi
    d2r = pi / 180
    
    # Move to bolt pickup position
    assembler.set_gripper_pos(180)
    assembler.move_to_pose([0.395110, -0.00355, 0.239044, -40.715475*d2r, 89.3258668*d2r, -39.975998*d2r])
    assembler.move_by_cartesian_path([0.395110, -0.00355, 0.219044, -40.715475*d2r, 89.3258668*d2r, -39.975998*d2r])
    assembler.set_gripper_pos(255)
    assembler.move_by_cartesian_path([0.395110, -0.00355, 0.239044, -40.715475*d2r, 89.3258668*d2r, -39.975998*d2r])

    # Move to bold drop position
    assembler.move_by_cartesian_path([0.243197, 0.124408, 0.548021, 0.000956, 0.762569, -0.005609])
    assembler.move_by_cartesian_path([0.243197, 0.174408, 0.548021, 0.000956, 0.762569, -0.005609])
    assembler.move_by_cartesian_path([0.243197, 0.234408, 0.548021, 0.000956, 0.762569, -0.005609])
    assembler.set_gripper_pos(0)
    assembler.move_by_cartesian_path([0.243197, 0.174408, 0.548021, 0.000956, 0.762569, -0.005609])

    assembler.move_to_init_pos()

    rospy.logwarn("Mission complete!")
