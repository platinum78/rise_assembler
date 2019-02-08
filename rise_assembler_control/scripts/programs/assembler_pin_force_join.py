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

    # Grip the pin,m
    assembler.set_gripper_pos(180)
    assembler.move_to_pose([0.258742, -0.00916, 0.341255, 65.778676*d2r, 89.337927*d2r, -24.88635*d2r])
    assembler.move_by_cartesian_path([0.258742, -0.00916, 0.241255, 65.778676*d2r, 89.337927*d2r, -24.88635*d2r])
    assembler.set_gripper_pos(255)
    assembler.move_by_cartesian_path([0.258742, -0.00916, 0.341255, 65.778676*d2r, 89.337927*d2r, -24.88635*d2r])

    # Move to temporary holding position
    assembler.move_to_pose([-0.21268, 0.441741, 0.301833, -163.117481*d2r, 89.33391020*d2r, -74.2842397*d2r])
    assembler.move_by_cartesian_path([-0.21268, 0.441741, 0.271833, -163.117481*d2r, 89.33391020*d2r, -74.2842397*d2r])
    assembler.set_gripper_pos(180)
    assembler.move_by_cartesian_path([-0.21268, 0.441741, 0.301833, -163.117481*d2r, 89.33391020*d2r, -74.2842397*d2r])
    assembler.set_gripper_pos(255)

    # Fix the pin
    assembler.move_to_pose([-0.21236, 0.205987, 0.244525, 0.7548351*d2r, 0.5168008*d2r, 90.027466*d2r])
    assembler.set_gripper_pos(0)
    assembler.move_by_cartesian_path([-0.21236, 0.205987, 0.040526, 0.7548351*d2r, 0.5168008*d2r, 90.027466*d2r])
    assembler.move_by_cartesian_path([-0.21236, 0.255988, 0.040527, 0.7548351*d2r, 0.5168008*d2r, 90.027466*d2r])
    assembler.set_gripper_pos(255)

    # Retract
    assembler.set_gripper_pos(0)
    assembler.move_by_cartesian_path([-0.21236, 0.205987, 0.040526, 0.7548351*d2r, 0.5168008*d2r, 90.027466*d2r])
    assembler.move_by_cartesian_path([-0.21236, 0.205987, 0.244525, 0.7548351*d2r, 0.5168008*d2r, 90.027466*d2r])

    # Go to initial position
    assembler.move_to_init_pos()

    rospy.logwarn("Mission complete!")