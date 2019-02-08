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
    assembler.set_gripper_pos(180)

    # Move first pin
    assembler.move_by_cartesian_path([0.239042, -0.158060, 0.300020, 0.920482, 1.570410, 0.920947])
    assembler.move_by_cartesian_path([0.239042, -0.158060, 0.240020, 0.920482, 1.570410, 0.920947])
    assembler.set_gripper_pos(255)
    assembler.move_by_cartesian_path([0.239042, -0.158060, 0.300020, 0.920482, 1.570410, 0.920947])

    # Drop 1st pin
    assembler.move_by_cartesian_path([0.559489, 0.032998, 0.323494, 1.536084, 1.569552, 1.536576])
    assembler.move_by_cartesian_path([0.559489, 0.032998, 0.263494, 1.536084, 1.569552, 1.536576])
    assembler.set_gripper_pos(180)
    assembler.move_by_cartesian_path([0.559489, 0.032998, 0.323494, 1.536084, 1.569552, 1.536576])


    # Move second pin
    assembler.move_by_cartesian_path([0.239042, -0.133060, 0.300020, 0.906049, 1.570410, 0.906520])
    assembler.move_by_cartesian_path([0.239042, -0.133060, 0.240020, 0.906049, 1.570410, 0.906520])
    assembler.set_gripper_pos(255)
    assembler.move_by_cartesian_path([0.239042, -0.133060, 0.300020, 0.906049, 1.570410, 0.906520])
    
    # Drop 2nd pin
    assembler.move_by_cartesian_path([0.552983, 0.063502, 0.328001, -2.588708, 1.570593, -2.587378])
    assembler.move_by_cartesian_path([0.552983, 0.063502, 0.268001, -2.588708, 1.570593, -2.587378])
    assembler.set_gripper_pos(180)
    assembler.move_by_cartesian_path([0.552983, 0.063502, 0.328001, -2.588708, 1.570593, -2.587378])

    # Move third pin
    assembler.move_by_cartesian_path([0.239033, -0.108058, 0.300016, 1.106348, 1.570430, 1.106852])
    assembler.move_by_cartesian_path([0.239033, -0.108058, 0.240016, 1.106348, 1.570430, 1.106852])
    assembler.set_gripper_pos(255)
    assembler.move_by_cartesian_path([0.239033, -0.108058, 0.300016, 1.106348, 1.570430, 1.106852])
    
    # Drop 3rd pin
    assembler.move_by_cartesian_path([0.517473, 0.217607, 0.325084, 0.719283, 1.569956, 0.719057])
    assembler.move_by_cartesian_path([0.517473, 0.217607, 0.265084, 0.719283, 1.569956, 0.719057])
    assembler.set_gripper_pos(180)
    assembler.move_by_cartesian_path([0.517473, 0.217607, 0.325084, 0.719283, 1.569956, 0.719057])

    # Move fourth pin
    assembler.move_by_cartesian_path([0.238017, -0.280022, 0.160018, 1.570814, 0.348179, 1.571369])
    assembler.move_by_cartesian_path([0.238017, -0.280022, 0.110018, 1.570814, 0.348179, 1.571369])
    assembler.set_gripper_pos(255)
    assembler.move_by_cartesian_path([0.238017, -0.280022, 0.160018, 1.570814, 0.348179, 1.571369])
    
    # Drop 4th pin
    assembler.move_by_cartesian_path([0.473999, 0.326471, 0.182539, 1.570736, 0.348005, 1.571350])
    assembler.move_by_cartesian_path([0.473999, 0.326471, 0.132539, 1.570736, 0.348005, 1.571350])
    assembler.set_gripper_pos(180)
    assembler.move_by_cartesian_path([0.473999, 0.326471, 0.182539, 1.570736, 0.348005, 1.571350])

    # Move fifth pin
    assembler.move_by_cartesian_path([0.238995, -0.253524, 0.160544, 1.570739, 0.347952, 1.571346])
    assembler.move_by_cartesian_path([0.238995, -0.253524, 0.110544, 1.570739, 0.347952, 1.571346])
    assembler.set_gripper_pos(255)
    assembler.move_by_cartesian_path([0.238995, -0.253524, 0.160544, 1.570739, 0.347952, 1.571346])
    
    # Drop 5th pin
    assembler.move_by_cartesian_path([0.469993, 0.295981, 0.180559, 1.570528, 0.347798, 1.571347])
    assembler.move_by_cartesian_path([0.469993, 0.295981, 0.130559, 1.570528, 0.347798, 1.571347])
    assembler.set_gripper_pos(180)
    assembler.move_by_cartesian_path([0.469993, 0.295981, 0.180559, 1.570528, 0.347798, 1.571347])

    # Move sixth pin
    assembler.move_by_cartesian_path([0.239039, -0.033058, 0.300019, 0.974383, 1.570409, 0.974852])
    assembler.move_by_cartesian_path([0.239039, -0.033058, 0.240019, 0.974383, 1.570409, 0.974852])
    assembler.set_gripper_pos(255)
    assembler.move_by_cartesian_path([0.239039, -0.033058, 0.300019, 0.974383, 1.570409, 0.974852])
    
    # Drop 6th pin
    assembler.move_by_cartesian_path([0.105963, 0.476885, 0.326000, -0.449636, 1.569878, -0.448566])
    assembler.move_by_cartesian_path([0.105963, 0.476885, 0.266000, -0.449636, 1.569878, -0.448566])
    assembler.set_gripper_pos(180)
    assembler.move_by_cartesian_path([0.105963, 0.476885, 0.326000, -0.449636, 1.569878, -0.448566])

    # Move 7th pin
    assembler.move_by_cartesian_path([0.239027, -0.008057, 0.300011, 1.162409, 1.570439, 1.162876])
    assembler.move_by_cartesian_path([0.239027, -0.008057, 0.240011, 1.162409, 1.570439, 1.162876])
    assembler.set_gripper_pos(255)
    assembler.move_by_cartesian_path([0.239027, -0.008057, 0.300011, 1.162409, 1.570439, 1.162876])
    
    # Drop 7th pin
    assembler.move_by_cartesian_path([0.104532, 0.510393, 0.319037, 1.575498, 1.569733, 1.575948])
    assembler.move_by_cartesian_path([0.104532, 0.510393, 0.259037, 1.575498, 1.569733, 1.575948])
    assembler.set_gripper_pos(180)
    assembler.move_by_cartesian_path([0.104532, 0.510393, 0.319037, 1.575498, 1.569733, 1.575948])

    # Move 8th pin
    assembler.move_by_cartesian_path([0.320076, -0.024023, 0.300004, -1.869397, 1.570002, -0.297776])
    assembler.move_by_cartesian_path([0.320076, -0.024023, 0.240004, -1.869397, 1.570002, -0.297776])
    assembler.set_gripper_pos(255)
    assembler.move_by_cartesian_path([0.320076, -0.024023, 0.300004, -1.869397, 1.570002, -0.297776])

    # Move to adjustment position
    assembler.move_by_cartesian_path([0.500073, 0.135960, 0.329915, -1.431298, 1.569131, 0.141131])
    assembler.move_by_cartesian_path([0.500073, 0.135960, 0.269915, -1.431298, 1.569131, 0.141131])
    assembler.set_gripper_pos(200)
    assembler.move_by_cartesian_path([0.540073, 0.135960, 0.269915, -1.431298, 1.569131, 0.141131])
    assembler.set_gripper_pos(255)
    
    # Drop 8th pin
    assembler.move_by_cartesian_path([0.363018, 0.245962, 0.249001, -1.574089, 0.752542, -0.010042])
    assembler.move_by_cartesian_path([0.363018, 0.245962, 0.199001, -1.574089, 0.752542, -0.010042])
    assembler.set_gripper_pos(180)
    assembler.move_by_cartesian_path([0.363018, 0.245962, 0.249001, -1.574089, 0.752542, -0.010042])


    # RODS ASSEMBLY
    ########################################

    

    assembler.move_to_init_pos()
    rospy.logwarn("Mission complete!")