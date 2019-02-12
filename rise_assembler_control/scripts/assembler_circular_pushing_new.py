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
from tf.transformations import quaternion_from_euler, euler_matrix, quaternion_from_matrix, euler_from_matrix
import numpy as np

def rotation_matrix_z(angle):
    return np.array([[np.cos(angle), -np.sin(angle), 0], [np.sin(angle), np.cos(angle), 0], [0, 0, 1]])

def circular_path(current_pose, radius, angle, direction, ee_length, path_resolution):
    # Find link 6 pose
    link6_pose = np.array(current_pose)
    orientation_matrix = euler_matrix(link6_pose[3], link6_pose[4], link6_pose[5])[0:3,0:3]
    v = np.matmul(orientation_matrix, np.array([1, 0, 0]))
    planar_angle = np.arctan2(v[1], v[0])
    
    slice_cnt = radius * abs(angle) / path_resolution
    angles = np.linspace(0, abs(angle), slice_cnt, True)
    if angle < 0:
        angles *= -1

    # Find 2-D pose of end-effector and pivot of revolution
    ee_position = link6_pose[0:3] + ee_length * np.array([np.cos(planar_angle), np.sin(planar_angle), 0])
    if direction == "ccw":
        rev_pivot = ee_position + radius * np.array([np.cos(planar_angle + np.pi/2), np.sin(planar_angle + np.pi/2), 0])
        pos_x = rev_pivot[0] + radius * np.cos(planar_angle - np.pi/2 + angles) - ee_length * np.cos(planar_angle)
        pos_y = rev_pivot[1] + radius * np.sin(planar_angle - np.pi/2 + angles) - ee_length * np.sin(planar_angle)
    elif direction == "cw":
        rev_pivot = ee_position + radius * np.array([np.cos(planar_angle - np.pi/2), np.sin(planar_angle - np.pi/2), 0])
        pos_x = rev_pivot[0] + radius * np.cos(planar_angle + np.pi/2 - angles) - ee_length * np.cos(planar_angle)
        pos_y = rev_pivot[1] + radius * np.sin(planar_angle + np.pi/2 - angles) - ee_length * np.sin(planar_angle)
    pos_z = np.array([rev_pivot[2]] * pos_x.shape[0])
    pos_x = np.reshape(pos_x, [1,-1])
    pos_y = np.reshape(pos_y, [1,-1])
    pos_z = np.reshape(pos_z, [1,-1])
    pos = np.append(np.append(pos_x, pos_y, axis=0), pos_z, axis=0).T
    pos = np.ndarray.tolist(pos)

    rpy = []
    if direction == "cw":
        angles *= -1
    for theta in angles:
        matrix = np.zeros([4,4])
        matrix[0:3,0:3] = np.matmul(rotation_matrix_z(theta), orientation_matrix)
        matrix[3,3] = 1
        rpy.append(euler_from_matrix(matrix))

    poses = []
    for p, q in zip(pos, rpy):
        pose = [0] * 6
        pose[0:3] = p[:]
        pose[3:6] = q[:]
        poses.append(pose)

    return poses

if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("moveit_test_node")

    assembler_cmd = moveit_commander.RobotCommander()
    abb_irb120 = moveit_commander.MoveGroupCommander("abb_irb120")
    assembler = rise_assembler_controller(assembler_cmd, abb_irb120)
    # assembler.move_to_test_pos()
    rospy.loginfo("SYSTEM READY!")


    poses = circular_path(current_pose=assembler.get_pose(), radius=0.2, angle=-np.pi/2, direction="ccw", ee_length=0.3, path_resolution=0.01)
    
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

