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
    rospy.loginfo("SYSTEM READY!!!")
    input_str = ""

    while True:
        input_str = raw_input("Type desired configuration >> ")
        print(input_str, "entered...")
        input_str = input_str.replace(",", "")
        
        # Quit operation
        if input_str in ["q", "Q"]:
            assembler.move_to_init_pos()
            break

        # Move to initial pose without quitting
        elif input_str in ["i", "I"]:
            assembler.move_to_init_pos()

        # Show current pose
        elif input_str in ["p", "P"]:
            p = assembler.abb_irb120.get_current_pose().pose
            o = p.orientation
            print p
            print "orientation_euler: "
            e = euler_from_quaternion([o.x, o.y, o.z, o.w])
            print "  r: ", e[0] * 180 / pi
            print "  p: ", e[1] * 180 / pi
            print "  y: ", e[2] * 180 / pi
            print "copy data: "
            print "  [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f]" % \
                (p.position.x, p.position.y, p.position.z, e[0], e[1], e[2])
            raw_input("Comments >> ")

        # Show current joint angles
        elif input_str in ["a", "A"]:
            angles = assembler.abb_irb120.get_current_joint_values()
            print "Radians: ", angles
            print "Degrees: ", [x * 180 / pi for x in angles]
            raw_input("Comments >> ")
        
        # Position controls
        else:
            input_data = input_str.split(" ")
            direction = input_data[0]
            pose_now = assembler.abb_irb120.get_current_pose().pose
            pose_new = copy.deepcopy(pose_now)

            # Cartesian position control
            if direction in ["x", "y", "z", "xyz"]:
                print("Linear hovering mode...")
                if direction == "x":
                    pose_new.position.x = pose_now.position.x + float(input_data[1])
                elif direction == "y":
                    pose_new.position.y = pose_now.position.y + float(input_data[1])
                elif direction == "z":
                    pose_new.position.z = pose_now.position.z + float(input_data[1])
                elif direction == "xyz":
                    pose_new.position.x = pose_now.position.x + float(input_data[1])
                    pose_new.position.y = pose_now.position.y + float(input_data[2])
                    pose_new.position.z = pose_now.position.z + float(input_data[3])
                
                pose = [0] * 6
                pose[0] = pose_new.position.x
                pose[1] = pose_new.position.y
                pose[2] = pose_new.position.z
                q = pose_new.orientation
                rpy = euler_from_quaternion([q.x, q.y, q.z, q.w])
                pose[3] = rpy[0]
                pose[4] = rpy[1]
                pose[5] = rpy[2]
                assembler.move_by_cartesian_path(pose)
            
            # Gripper position settings
            elif direction in ["g", "G"]:
                print("Gripper manipulation mode...")
                assembler.set_gripper_pos(int(input_data[1]))
            
            # Joint angle mode
            elif direction in ["j", "J"]:
                print("Joint revolution mode...")
                joint_angles = assembler.abb_irb120.get_current_joint_values()
                joint_angles[int(input_data[1])-1] += float(input_data[2]) * pi / 180
                assembler.move_to_joint_pos(joint_angles)
            
            # Configuration mode
            elif direction in ["c", "C"]:
                print("Move-to-configuration mode...")
                pose = [0] * 6
                pose[0:3] = [float(x) for x in input_data[1:4]]
                pose[3:6] = [float(x) * pi / 180 for x in input_data[4:7]]
                assembler.move_to_pose(pose)
            
            elif direction in ["cr", "cR", "Cr", "CR"]:
                print("Move-to-configuration mode with radian units...")
                pose = [0] * 6
                pose[0:3] = [float(x) for x in input_data[1:4]]
                pose[3:6] = [float(x) for x in input_data[4:7]]
                assembler.move_to_pose(pose)
            
            # Translation mode
            elif direction in ["t", "T"]:
                print("Translation mode...")
                pose = [0] * 6
                pose[0:3] = [float(x) for x in input_data[1:4]]
                q = assembler.abb_irb120.get_current_pose().pose.orientation
                rpy = euler_from_quaternion([q.x, q.y, q.z, q.w])
                pose[3], pose[4], pose[5] = rpy[0], rpy[1], rpy[2]
                assembler.move_by_cartesian_path(pose)
            
            # Orientation mode
            elif direction in ["o", "O"]:
                print("Orientation setting mode...")
                p = assembler.abb_irb120.get_current_pose().pose.position
                pose = [0] * 6
                pose[0], pose[1], pose[2] = p.x, p.y, p.z
                pose[3], pose[4], pose[5] = [float(x) * pi / 180 for x in input_data[1:4]]
                assembler.move_to_pose(pose)