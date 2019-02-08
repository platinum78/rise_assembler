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
from tf.transformations import quaternion_from_euler
import numpy as np

if __name__ == "__main__":
    circle_segments = 40
    # Initialize ROS and MoveIt!
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("moveit_test_node")
    assembler = moveit_commander.RobotCommander()
    abb_irb120 = moveit_commander.MoveGroupCommander("abb_irb120")
    
    theta = np.linspace(0, np.pi*4, circle_segments).tolist()
    x = 0.05 * (1 - np.cos(theta).reshape([-1,1]))
    y = np.linspace(0, 0.0024375 * 2, circle_segments).reshape([-1,1])
    z = 0.05 * np.sin(theta).reshape([-1,1])
    points = np.append(np.append(x, y, axis=1), z, axis=1).tolist()
    
    for idx in range(3):
        pose_current = abb_irb120.get_current_pose().pose
        poses = []
        for point in points:
            pose_circle = copy.deepcopy(pose_current)
            pose_circle.position.x += point[0]
            pose_circle.position.y += point[1]
            pose_circle.position.z += point[2]
            poses.append(pose_circle)

        (plan, fraction) = abb_irb120.compute_cartesian_path(poses, 0.015, 0.0)
        plan.joint_trajectory.points.pop(0)

        
        abb_irb120.execute(plan, wait=True)

    abb_irb120.go(pose_current, wait=True)
    # print abb_irb120.plan(pose_target)