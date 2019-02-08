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
    # Gripping experiments
    # Open gripper
    assembler.set_gripper_pos(150)
    
    # Grasp the oblique rod
    assembler.move_to_pose([0.500608, 0.148911, 0.357657, 103.84343*d2r, 89.706543*d2r, 103.52193*d2r])
    assembler.move_by_cartesian_path([0.500608, 0.148911, 0.257657, 103.84343*d2r, 89.706543*d2r, 103.52193*d2r])
    assembler.set_gripper_pos(255)
    assembler.move_by_cartesian_path([0.500608, 0.148911, 0.407657, 103.84343*d2r, 89.706543*d2r, 103.52193*d2r])
    
    # Align the oblique rod
    assembler.set_gripper_pos(180)
    assembler.move_to_pose([0.501827, 0.008583, 0.333544, -77.13002*d2r, 89.148416*d2r, -167.3905*d2r])
    assembler.move_by_cartesian_path([0.501827, 0.008583, 0.233544, -77.13002*d2r, 89.148416*d2r, -167.3905*d2r])
    assembler.set_gripper_pos(255)
    assembler.move_by_cartesian_path([0.501827, 0.008583, 0.433544, -77.13002*d2r, 89.148416*d2r, -167.3905*d2r])
    
    # Put the rod onto front slot
    assembler.move_to_pose([0.262244, 0.054372, 0.227679, -92.51962*d2r, 0.7504137*d2r, -0.519940*d2r])
    assembler.move_by_cartesian_path([0.262244, 0.054372, 0.167679, -92.51962*d2r, 0.7504137*d2r, -0.519940*d2r])
    assembler.move_by_cartesian_path([0.262244, 0.054372, 0.127679, -92.51962*d2r, 0.7504137*d2r, -0.519940*d2r])
    assembler.set_gripper_pos(150)
    assembler.move_by_cartesian_path([0.262244, 0.054372, 0.427679, -92.51962*d2r, 0.7504137*d2r, -0.519940*d2r])

    assembler.move_to_init_pos()

    rospy.logwarn("Mission complete!")