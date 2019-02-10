#!/usr/bin/python2

import os, sys
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
print BASE_DIR
sys.path.append(BASE_DIR)

from assembler_controller import *
from assembler_tasks import *
from points_setup import *

import sys, copy, rospy, moveit_commander, moveit_msgs.msg, geometry_msgs.msg, time, math
from math import pi, sqrt
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler
import numpy as np
import assembler_circular_pushing_new

# Delete this after testing
from matplotlib import pyplot as plt

pi = 3.141592653589793
pose_from_abb = [1,2,3,4,5,0]
def radius_angle_to_pose(pose_from_abb, radius, angle):
    d_angle = angle/10
    cur_pose = pose_from_abb
    rot_center = [pose_from_abb[0]+radius*math.cos(pose_from_abb[5]-pi/2), \
                  pose_from_abb[1]+radius*math.sin(pose_from_abb[5]-pi/2)]
    pose =  [[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0]]
    print "current position is: [", cur_pose[0],", ", cur_pose[1],"]"
    print "rotation center is: ", rot_center
    
    poses = []
    for i in range(10):
        pose[i] = [math.cos(d_angle)*(cur_pose[0] - rot_center[0]) + math.sin(d_angle)*(cur_pose[1] - rot_center[1])+rot_center[0], 
            -math.sin(d_angle)*(cur_pose[0] - rot_center[0]) + math.cos(d_angle)*(cur_pose[1] - rot_center[1])+rot_center[1]]
        pose_from_abb[5] = angle*i/10 + d_angle 
        cur_pose = [pose[i][0], pose[i][1], pose_from_abb[2], pose_from_abb[3], pose_from_abb[4], pose_from_abb[5]]
        poses.append(cur_pose)
    
    # poses_tensor = np.array(poses)
    # plt.figure(1)
    # plt.plot(poses_tensor[:,5])
    # plt.scatter(poses_tensor[:,0], poses_tensor[:,1])
    # plt.xlim([-1, 1])
    # plt.ylim([-1, 1])
    plt.show()
    return poses

if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("moveit_test_node")

    assembler_cmd = moveit_commander.RobotCommander()
    abb_irb120 = moveit_commander.MoveGroupCommander("abb_irb120")
    assembler = rise_assembler_controller(assembler_cmd, abb_irb120)
    # assembler.move_to_test_pos()
    rospy.loginfo("SYSTEM READY!")

    pose_from_abb = assembler.get_pose()
    print pose_from_abb

    poses = radius_angle_to_pose(pose_from_abb, 0.15, -pi/2)

    # for pose in poses:
    #     pose[0] *= -1
    #     pose[1] *= -1

    for pose in poses:
        print pose
        assembler.move_by_cartesian_path(pose)
    