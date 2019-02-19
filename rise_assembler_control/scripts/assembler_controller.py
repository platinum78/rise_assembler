#!/usr/bin/python2
import sys, copy, rospy, moveit_commander, moveit_msgs.msg, geometry_msgs.msg, time
from math import pi, sqrt
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_matrix
import numpy as np

import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
import rospy
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg

class rise_assembler_controller:
    def __init__(self, assembler, abb_irb120):
        # Initialize MoveIt! for ABB IRB 120
        self.assembler = assembler
        self.abb_irb120 = abb_irb120
        self.joint_angle_init_pose = [0, 0, 0, 0, pi/2, 0]
        self.joint_angle_goal_prev = [0, 0, 0, 0, pi/2, 0]
        self.wpose = geometry_msgs.msg.Pose()
        self.was_previous_command_execute = False
        self.abb_irb120.set_goal_joint_tolerance(0.01)
        self.gripper_open_pos = 150
        self.gripper_grip_pos = 255
        # self.abb_irb120.set_goal_position_tolerance(0.01)
        # self.abb_irb120.set_goal_orientation_tolerance(0.01)

        # Initialize ROS publisher units for gripper manipulation.
        self.gripper_pub = rospy.Publisher('Robotiq2FGripperRobotOutput', \
                            outputMsg.Robotiq2FGripper_robot_output)
        self.gripper_command = outputMsg.Robotiq2FGripper_robot_output();
        self.gripper_command.rACT = 1
        self.gripper_command.rGTO = 1
        self.gripper_command.rATR = 0
        self.gripper_command.rPR = 255
        self.gripper_command.rSP = 255
        self.gripper_command.rFR = 255

    # Shortcut Function: move to initial position
    def move_to_init_pos(self):
        # Move to non-singularity position
        # self.joint_angle_goal = copy.deepcopy(self.joint_angle_pos)
        self.abb_irb120.stop()
        self.abb_irb120.clear_pose_targets()

        rospy.loginfo("Moving to initial pose...")
        self.abb_irb120.set_joint_value_target(self.joint_angle_init_pose)
        self.abb_irb120.go(wait=True)
        rospy.loginfo("Moved to initial pose.")
        time.sleep(1.0)
        
    
    def move_to_pose(self, pose):
        x, y, z, r, p, y_ = pose
        rospy.loginfo("move_to_pose() called.")
        self.abb_irb120.stop()
        self.abb_irb120.clear_pose_targets()
        self.wpose = geometry_msgs.msg.Pose()

        self.wpose.position.x = x
        self.wpose.position.y = y
        self.wpose.position.z = z

        self.q = quaternion_from_euler(r, p, y_)
        self.wpose.orientation.x = self.q[0]
        self.wpose.orientation.y = self.q[1]
        self.wpose.orientation.z = self.q[2]
        self.wpose.orientation.w = self.q[3]

        self.abb_irb120.set_pose_target(self.wpose)
        self.abb_irb120.go(wait=True)

    def move_by_cartesian_path(self, pose, path_resolution=0.01):
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        # self.abb_irb120.clear_pose_targets()
        x, y, z, r, p, y_ = pose
        self.abb_irb120.stop()
        self.abb_irb120.clear_pose_targets()

        self.waypoints = []
        self.wpose = self.abb_irb120.get_current_pose().pose
        self.waypoints.append(copy.deepcopy(self.wpose))
        
        self.wpose.position.x = x
        self.wpose.position.y = y
        self.wpose.position.z = z

        self.q = quaternion_from_euler(r, p, y_)
        self.wpose.orientation.x = self.q[0]
        self.wpose.orientation.y = self.q[1]
        self.wpose.orientation.z = self.q[2]
        self.wpose.orientation.w = self.q[3]
        self.waypoints.append(copy.deepcopy(self.wpose))
        self.abb_irb120.set_pose_target(self.wpose)

        rospy.loginfo("Computing path...")
        (trajectory_plan, fraction) = self.abb_irb120.compute_cartesian_path(self.waypoints, path_resolution, 0.0)
        trajectory_plan.joint_trajectory.points.pop(0)
        
        rospy.loginfo("Moving on Cartesian path...")
        self.abb_irb120.execute(trajectory_plan, wait=True)
        rospy.loginfo("Subtly adjusting final position...")
        self.abb_irb120.go(wait=True)
        rospy.loginfo("Movement finished!")
        # rospy.logwarn("Movement finished!")
    
    def get_joint_angles(self, log=True):
        self.joint_angle_pos = self.abb_irb120.get_current_joint_values()
        if log:    
            angles_string = "%4f, %4f, %4f, %4f, %4f, %4f" % \
                            tuple(self.joint_angle_pos[x] for x in range(6))
            rospy.loginfo("Current joint angles: " + angles_string)
    
    def move_to_test_pos(self):
        joint_angle_goal = [0.3, 0.3, 0.3, 0.3, 0.3, 0.3]
        self.abb_irb120.go(joint_angle_goal, wait=True)
    
    def move_to_joint_pos(self, pos):
        joint_angle_goal = pos
        self.abb_irb120.go(joint_angle_goal, wait=True)
    
    def set_gripper_pos(self, pos):
        self.gripper_command.rPR = int(pos)
        self.gripper_pub.publish(self.gripper_command)
        time.sleep(1.5)
    
    def gripper_open(self):
        self.set_gripper_pos(self.gripper_open_pos)
    
    def gripper_grip(self):
        self.set_gripper_pos(self.gripper_grip_pos)
    
    def rotate_joint(self, joint_idx, angle):
        self.get_joint_angles()
        joint_angle_goal = self.joint_angle_pos
        joint_angle_goal[joint_idx-1] += angle

        abb_irb120.stop()
        abb_irb120.set_joint_value_target(joint_angle_goal)
        abb_irb120.go(wait=True)
        rospy.loginfo("Joint %d rotated by %6f radians." % (joint_idx+1, angle))
    
    def get_pose(self):
        pose = self.abb_irb120.get_current_pose().pose
        q = pose.orientation
        p = pose.position
        rpy = euler_from_quaternion([q.x, q.y, q.z, q.w])
        pose_list = [0] * 6
        pose_list[0] = p.x
        pose_list[1] = p.y
        pose_list[2] = p.z
        pose_list[3] = rpy[0]
        pose_list[4] = rpy[1]
        pose_list[5] = rpy[2]
        return pose_list
    
    def move_by_circular_path(radius, angle, ee_length, direction, path_resolution):
        pose = self.abb_irb120.get_current_pose().pose
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        q = pose.orientation
        rpy = euler_from_quaternion([q.x, q.y, q.z, q.w])
        rotation_matrix = quaternion_matrix([q.x, q.y, q.z, q.w])
        print rotation_matrix
        v = np.matmul(rotation_matrix, np.array([[1], [0], [0], [0]]))
        print v
        plane_angle = np.arctan2(v[1,0], v[0,0])

        ########################################
        # PART TO SETUP
        ########################################

        hand_inclination = 10       # Given in degrees
        circle_direction = "ccw"
        hand_length = 0.2
        path_resolution = 0.01

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