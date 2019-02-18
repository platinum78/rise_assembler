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
from geometry_msgs.msg import Quaternion, Pose
from tf.transformations import quaternion_from_euler

import numpy as np
from assembler_circular_pushing_new import circular_path

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
    # moveit_commander.roscpp_initialize(sys.argv)
    # rospy.init_node("moveit_test_node")

    # assembler_cmd = moveit_commander.RobotCommander()
    # abb_irb120 = moveit_commander.MoveGroupCommander("abb_irb120")
    # assembler = rise_assembler_controller(assembler_cmd, abb_irb120)
    # # assembler.move_to_test_pos()
    # rospy.loginfo("SYSTEM READY!")

    # pose_from_abb = assembler.get_pose()
    # print pose_from_abb

    # poses = radius_angle_to_pose(pose_from_abb, 0.15, -pi/2)

    # # for pose in poses:
    # #     pose[0] *= -1
    # #     pose[1] *= -1

    # for pose in poses:
    #     print pose
    #     assembler.move_by_cartesian_path(pose)

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("moveit_test_node")

    assembler_cmd = moveit_commander.RobotCommander()
    abb_irb120 = moveit_commander.MoveGroupCommander("abb_irb120")
    assembler = rise_assembler_controller(assembler_cmd, abb_irb120)
    # assembler.move_to_test_pos()
    rospy.loginfo("SYSTEM READY!")


    ## translation before rotation

    # pose_from_abb = assembler.get_pose()
    # pose_from_abb[0] = pose_from_abb[0] + 0.3
    # assembler.move_by_cartesian_path(pose_from_abb)

    ## rotation
    rotation_angle1 = np.pi/6
    rotation_radius1 = 0.25 #min 0.25
    poses = circular_path(assembler.get_pose(), rotation_radius1, rotation_angle1, "ccw", 0.215, 0.01)
    
    waypoints = []
    for pose in poses:
        p = Pose()
        q = quaternion_from_euler(pose[3], pose[4], pose[5])
        p.position.x = pose[0]
        p.position.y = pose[1]
        p.position.z = pose[2]
        p.orientation.x = q[0]
        p.orientation.y = q[1]
        p.orientation.z = q[2]
        p.orientation.w = q[3]
        waypoints.append(p)
    
    (trajectory_plan, fraction) = assembler.abb_irb120.compute_cartesian_path(waypoints, 0.005, 0.0)
    trajectory_plan.joint_trajectory.points.pop(0)
    
    rospy.loginfo("Moving on Cartesian path...")
    assembler.abb_irb120.execute(trajectory_plan, wait=True)

    # #translation to next motion

    pose_from_abb = assembler.get_pose()
    pose_from_abb[0] = pose_from_abb[0] - 0.03*np.cos(rotation_angle1)
    pose_from_abb[1] = pose_from_abb[1] - 0.03*np.sin(rotation_angle1) 
    assembler.move_by_cartesian_path(pose_from_abb)

    pose_from_abb = assembler.get_pose()
    # pose_from_abb[0] = pose_from_abb[0] - (0.215 + 0.0335)*np.sin(rotation_angle) \
    #     + (0.0335 + 0.03)*np.sin(rotation_angle) + 0.063
    pose_from_abb[0] = pose_from_abb[0] - ((0.0335 + 0.03 + 0.215)*(1 - np.cos(rotation_angle1)))
    pose_from_abb[1] = pose_from_abb[1] + (0.0355 + 0.03 + 0.215)*np.sin(rotation_angle1) 
    pose_from_abb[5] = pose_from_abb[5] - rotation_angle1
    assembler.move_by_cartesian_path(pose_from_abb)

    pose_from_abb = assembler.get_pose()
    pose_from_abb[0] = pose_from_abb[0] + 0.03
    assembler.move_by_cartesian_path(pose_from_abb)
    
    # # #translation when the rotation is one time

    # pose_from_abb = assembler.get_pose()
    # pose_from_abb[0] = pose_from_abb[0] + 0.7 - rotation_radius1
    # pose_from_abb[1] = pose_from_abb[1] + 0.1 - rotation_radius1 + rotation_radius1*np.cos(rotation_angle1)
    # assembler.move_by_cartesian_path(pose_from_abb)

    #translation when the rotation is two times

    pose_from_abb = assembler.get_pose()
    pose_from_abb[0] = pose_from_abb[0] + 0.25
    pose_from_abb[1] = pose_from_abb[1] - 0.25*np.cos(rotation_angle1)
    assembler.move_by_cartesian_path(pose_from_abb)

    ## second rotation
    rotation_angle2 = np.pi/4
    rotation_radius2 = 0.25 
    poses = circular_path(assembler.get_pose(), rotation_radius2, rotation_angle2, "ccw", 0.215, 0.01)
    
    waypoints = []
    for pose in poses:
        p = Pose()
        q = quaternion_from_euler(pose[3], pose[4], pose[5])
        p.position.x = pose[0]
        p.position.y = pose[1]
        p.position.z = pose[2]
        p.orientation.x = q[0]
        p.orientation.y = q[1]
        p.orientation.z = q[2]
        p.orientation.w = q[3]
        waypoints.append(p)
    
    (trajectory_plan, fraction) = assembler.abb_irb120.compute_cartesian_path(waypoints, 0.005, 0.0)
    trajectory_plan.joint_trajectory.points.pop(0)
    
    rospy.loginfo("Moving on Cartesian path...")
    assembler.abb_irb120.execute(trajectory_plan, wait=True)