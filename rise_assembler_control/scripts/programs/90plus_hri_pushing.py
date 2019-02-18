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
    # assembler.move_to_test_pos()
    rospy.loginfo("SYSTEM READY!")

    #assembler.move_to_init_pos()
    #assembler.set_gripper_pos(180)
    assembler.move_by_cartesian_path([-0.153797, 0.414940, -0.006671, -1.557638, 0.189843, -0.002580])
    assembler.move_by_cartesian_path([-0.053813, 0.424913, -0.006653, -1.557704, 0.189882, -0.002797])
    #assembler.set_gripper_pos(255)
    #assembler.move_by_cartesian_path([0.246211, 0.454944, -0.006677, -1.557616, 0.189907, -0.002457])

    #assembler.move_to_pose([0.315475, -0.325656, 0.283483, -1.571006, 1.169316, -0.000112]) 
    assembler.move_by_cartesian_path([-0.058990, 0.424961, -0.006636, -1.557724, 0.189834, -0.002768])  #x backwards
    assembler.move_by_cartesian_path([-0.058885, 0.407817, -0.006646, -1.557646, 0.190002, -0.002817])  #mid pose
    
    assembler.move_by_cartesian_path([0.043215, 0.417815, -0.006648, -1.557647, 0.190004, -0.002818])  #second rotation
    #assembler.set_gripper_pos(180)
    assembler.move_by_cartesian_path([0.038115, 0.417815, -0.006648, -1.557647, 0.190004, -0.002818])
    #assembler.set_gripper_pos(255)
    assembler.move_by_cartesian_path([0.038215, 0.404815, -0.006648, -1.557647, 0.190004, -0.002818])

    assembler.move_by_cartesian_path([0.141215, 0.414815, -0.006648, -1.557655, 0.189999, -0.002814])  #third rotation

    assembler.move_by_cartesian_path([0.136215, 0.414815, -0.006648, -1.557655, 0.189999, -0.002814])
    assembler.move_by_cartesian_path([0.137215, 0.398815, -0.006648, -1.557655, 0.189999, -0.002814])

    assembler.move_by_cartesian_path([0.241214, 0.418806, -0.006659, -1.557667, 0.190009, -0.002821])  #fourth rotation
    #assembler.move_by_cartesian_path([0.241214, 0.429806, -0.006659, -1.557667, 0.190009, -0.002821])
    #assembler.move_by_cartesian_path([0.241214, 0.429806, -0.006659, -1.557667, 0.190009, -0.002821])
    #assembler.move_by_cartesian_path([0.241214, 0.429806, -0.006659, -1.557667, 0.190009, -0.002821])














    #assembler.move_to_init_pos()
