#!/usr/bin/python2

import sys, copy, rospy, moveit_commander, moveit_msgs.msg, geometry_msgs.msg, time
from math import pi, sqrt
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler
import numpy as np

class rise_assembler_controller:
    def __init__(self, assembler, abb_irb120):
        self.assembler = assembler
        self.abb_irb120 = abb_irb120
        self.joint_angle_init_pose = [0, 0, 0, 0, pi/2, 0]
        self.joint_angle_goal_prev = [0, 0, 0, 0, pi/2, 0]
        self.wpose = geometry_msgs.msg.Pose()
        self.was_previous_command_execute = False
        self.abb_irb120.set_goal_joint_tolerance(0.01)
        # self.abb_irb120.set_goal_position_tolerance(0.01)
        # self.abb_irb120.set_goal_orientation_tolerance(0.01)

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

    
    def move_to_pose(self, x, y, z, r, p, y_):
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
        time.sleep(0.5)

    def move_by_cartesian_path(self, x, y, z, r, p, y_, path_resolution=0.01):
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        # self.abb_irb120.clear_pose_targets()
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
        (trajectory_plan, fraction) = abb_irb120.compute_cartesian_path(self.waypoints, path_resolution, 0.0)
        trajectory_plan.joint_trajectory.points.pop(0)
        
        rospy.loginfo("Moving on Cartesian path...")
        self.abb_irb120.execute(trajectory_plan, wait=True)
        rospy.loginfo("Subtly adjusting final position...")
        self.abb_irb120.go(wait=True)
        # rospy.logwarn("Movement finished!")
        time.sleep(0.5)
    
    def get_joint_angles(self, log=True):
        self.joint_angle_pos = self.abb_irb120.get_current_joint_values()
        if log:    
            angles_string = "%4f, %4f, %4f, %4f, %4f, %4f" % \
                            tuple(self.joint_angle_pos[x] for x in range(6))
            rospy.loginfo("Current joint angles: " + angles_string)
    
    def move_to_test_pos(self):
        joint_angle_goal = [0.3, 0.3, 0.3, 0.3, 0.3, 0.3]
        abb_irb120.go(joint_angle_goal, wait=True)
    

if __name__ == "__main__":
    
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("moveit_test_node")
    assembler_cmd = moveit_commander.RobotCommander()
    abb_irb120 = moveit_commander.MoveGroupCommander("abb_irb120")
    assembler = rise_assembler_controller(assembler_cmd, abb_irb120)
    # assembler.move_to_test_pos()
    try:
        base_height = 0.0
        rospy.logwarn("Moving to initial position...")
        assembler.move_to_init_pos()
        rospy.logwarn("Moving to part position...")
        assembler.move_to_pose(0, -0.5, 0.3 + base_height, 0, pi/2, -pi/2)
        rospy.logwarn("Descending to pick part...")
        assembler.move_by_cartesian_path(0, -0.5, 0.25 + base_height, 0, pi/2, -pi/2, path_resolution=0.005)
        rospy.logwarn("Picked part up. Now ascending...")
        assembler.move_by_cartesian_path(0, -0.5, 0.3 + base_height, 0, pi/2, -pi/2, path_resolution=0.005)
        rospy.logwarn("Moving to desntination...")
        assembler.move_to_pose(0.4, 0, 0.4 + base_height, 0, 0, 0)
        rospy.logwarn("Arrived to destination. Pulling part into slot...")
        assembler.move_by_cartesian_path(0.45, 0, 0.4 + base_height, 0, 0, 0, path_resolution=0.005)
        rospy.logwarn("Pulled part into slot. Now withdrawing from workplace...")
        assembler.move_by_cartesian_path(0.4, 0, 0.4 + base_height, 0, 0, 0, path_resolution=0.005)
        rospy.logwarn("Mission complete. Now returning to initial pose...")
        assembler.move_to_init_pos()
        rospy.logwarn("Motion complete!")
    except rospy.ROSInterruptException:
        pass