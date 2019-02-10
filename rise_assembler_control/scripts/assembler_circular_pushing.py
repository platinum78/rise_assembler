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
from moveit_msgs.msg import PositionIKRequest
from geometry_msgs.msg import Quaternion, Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_matrix
import numpy as np

def circular_path(x_, y_, theta_, move_angle, radius, hand_length, direction, path_resolution=0.01):
    # Calculate number of path slices needed to keep path resolution
    arc_length = move_angle * radius
    slice_cnt = int(arc_length / path_resolution)
    x, y = x_ + hand_length * np.cos(theta_), y_ + hand_length * np.sin(theta_)
    
    if direction == "ccw":
        x_0, y_0 = x + radius * np.cos(theta_ + np.pi / 2), y + radius * np.sin(theta_ + np.pi / 2)
        theta_0 = theta_
        theta_r = np.linspace(theta_0, theta_0 + move_angle, slice_cnt, True)
        theta = theta_r - np.pi / 2
    elif direction == "cw":
        x_0, y_0 = x + radius * np.cos(theta_ - np.pi / 2), y + radius * np.sin(theta_ - np.pi / 2)
        theta_0 = theta_
        theta_r = np.linspace(theta_0, theta_0 - move_angle, slice_cnt, True)
        theta = theta_r + np.pi / 2
    
    
    # Calculate the circular-path points of the end-effector
    x_e = x_0 + radius * np.cos(theta)
    y_e = y_0 + radius * np.sin(theta)
    
    x_r = x_e - hand_length * np.cos(theta_r)
    y_r = y_e - hand_length * np.sin(theta_r)
    
    return x_r, y_r, theta_r

if __name__ == "__main__":
    circle_segments = 40
    # Initialize ROS and MoveIt!
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("moveit_test_node")
    assembler = moveit_commander.RobotCommander()
    abb_irb120 = moveit_commander.MoveGroupCommander("abb_irb120")

    pose_current = abb_irb120.get_current_pose().pose
    x = pose_current.position.x
    y = pose_current.position.y
    z = pose_current.position.z
    q = pose_current.orientation
    rpy = euler_from_quaternion([q.x, q.y, q.z, q.w])
    r = rpy[0]
    p = rpy[1]
    y_ = rpy[2]
    rotation_matrix = quaternion_matrix([q.x, q.y, q.z, q.w])
    print rotation_matrix
    v = np.matmul(rotation_matrix, np.array([[1], [0], [0], [0]]))
    print v
    plane_angle = np.arctan2(v[1,0], v[0,0])

    ########################################
    # PART TO SETUP
    ########################################

    hand_inclination = 10       # Given in degrees
    move_angle = 10             # Given in degrees
    circle_direction = "ccw"
    hand_length = 0.2
    path_radius = 0.2
    path_resolution = 0.01

    ########################################
    ########################################
    print plane_angle * 180 / np.pi
    x_r, y_r, theta_r = circular_path(x_=x, y_=x, theta_=plane_angle, move_angle=move_angle*np.pi/180, radius=path_radius, \
                                      hand_length=hand_length, direction=circle_direction,  path_resolution=path_resolution)
    x_r = np.ndarray.tolist(x_r)
    y_r = np.ndarray.tolist(y_r)
    theta_r = np.ndarray.tolist(theta_r)
    
    
    pose_current = abb_irb120.get_current_pose().pose
    poses = []
    for x, y, theta in zip(x_r, y_r, theta_r):
        pose_circle = copy.deepcopy(pose_current)
        q = quaternion_from_euler(r, p, theta)
        pose_circle.position.x = x
        pose_circle.position.y = y
        pose_circle.position.z = z
        pose_circle.orientation.x = q[0]
        pose_circle.orientation.y = q[1]
        pose_circle.orientation.z = q[2]
        pose_circle.orientation.w = q[3]
        poses.append(pose_circle)
    print "Current pose: "
    print pose_current
    print ""
    print "Planned poses: "
    for pose in poses:
        print pose, "\n"

    (plan, fraction) = abb_irb120.compute_cartesian_path(poses, 0.01, 0.0)
    plan.joint_trajectory.points.pop(0)

        
    abb_irb120.execute(plan, wait=True)
    abb_irb120.go(pose_current, wait=True)
    # print abb_irb120.plan(pose_target)