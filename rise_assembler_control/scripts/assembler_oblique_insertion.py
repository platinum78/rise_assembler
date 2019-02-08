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

    assembler.move_to_init_pos()
    assembler.set_gripper_pos(180)
    assembler.move_by_cartesian_path([0.387480, 0.023270, 0.307650, 2.869432, 1.554345, 2.858716])
    assembler.move_by_cartesian_path([0.387480, 0.023270, 0.257650, 2.869432, 1.554345, 2.858716])
    assembler.set_gripper_pos(255)
    assembler.move_by_cartesian_path([0.387480, 0.023270, 0.307650, 2.869432, 1.554345, 2.858716])

    assembler.move_to_pose([0.315475, -0.325656, 0.283483, -1.571006, 1.169316, -0.000112])
    assembler.move_by_cartesian_path([0.415475, -0.325656, 0.283483, -1.571006, 1.16931, -0.000112])
    assembler.move_by_cartesian_path([0.418458, -0.325641, 0.275433, -1.571204, 1.169358, -0.000233])
    assembler.move_by_cartesian_path([0.420453, -0.325626, 0.272414, -1.571185, 1.169413, -0.000195])
    assembler.set_gripper_pos(180)
    assembler.move_by_cartesian_path([0.420453, -0.325626, 0.282414, -1.571185, 1.169413, -0.000195])
    assembler.set_gripper_pos(255)
    assembler.move_by_cartesian_path([0.420453, -0.325626, 0.273300, -1.571185, 1.169413, -0.000195])

    assembler.move_to_init_pos()