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


    # PINS ASSEMBLY
    ########################################
    points = []
    for pose_couple in task_poses["rods_straight"]:
        points.append(grasp_points_msg(pose_couple))
    
    rospy.logwarn("Moving to initial pose...")
    assembler.move_to_init_pos()

    for idx in range(len(points)):
        rospy.logwarn("Moving pin #%d..." % idx)
        grasp_and_move(assembler, points[idx])

    rospy.logwarn("Moving to initial pose...")
    assembler.move_to_init_pos()

    rospy.logwarn("Mission complete!")