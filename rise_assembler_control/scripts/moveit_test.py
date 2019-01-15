#!/usr/bin/python2

import sys, copy, rospy, moveit_commander, moveit_msgs.msg, geometry_msgs.msg, time
from math import pi, sqrt
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler
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
        self.gripper_open = 150
        self.gripper_grip = 255
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
    
    def set_gripper_pos(self, pos):
        self.gripper_command.rPR = int(pos)
        self.gripper_pub.publish(self.gripper_command)
        time.sleep(1.5)
    

def transfer_pin(assembler, start, end):
    grip_height = 0.05
    x_offset = 0.005
    rospy.logwarn("Moving to pickup position...")
    assembler.move_to_pose(start[0] + x_offset, start[1], start[2] + grip_height, start[3], start[4], start[5])
        
    rospy.logwarn("Opening gripper...")
    assembler.set_gripper_pos(assembler.gripper_open)

    rospy.logwarn("Descending to pick part...")
    assembler.move_by_cartesian_path(start[0] + x_offset, start[1], start[2], start[3], start[4], start[5], path_resolution=0.005)
        
    rospy.logwarn("Closing gripper...")
    assembler.set_gripper_pos(assembler.gripper_grip)
        
    rospy.logwarn("Picked part up. Now ascending...")
    assembler.move_by_cartesian_path(start[0] + x_offset, start[1], start[2] + grip_height, start[3], start[4], start[5], path_resolution=0.005)
        
    rospy.logwarn("Moving to desntination...")
    assembler.move_to_pose(end[0] + x_offset, end[1], end[2] + grip_height, end[3], end[4], end[5])
        
    rospy.logwarn("Arrived to destination. Pulling part into slot...")
    assembler.move_by_cartesian_path(end[0] + x_offset, end[1], end[2], end[3], end[4], end[5], path_resolution=0.005)
        
    rospy.logwarn("Opening gripper...")
    assembler.set_gripper_pos(assembler.gripper_open)
        
    rospy.logwarn("Pulled part into slot. Now withdrawing from workplace...")
    assembler.move_by_cartesian_path(end[0] + x_offset, end[1], end[2] + grip_height, end[3], end[4], end[5], path_resolution=0.005)

    rospy.logwarn("Closing gripper...")
    assembler.set_gripper_pos(255)

    rospy.logwarn("Cycle complete.")


if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("moveit_test_node")

    assembler_cmd = moveit_commander.RobotCommander()
    abb_irb120 = moveit_commander.MoveGroupCommander("abb_irb120")
    assembler = rise_assembler_controller(assembler_cmd, abb_irb120)
    # assembler.move_to_test_pos()
    rospy.loginfo("SYSTEM READY!!!")

    points = [
        [(-0.213481754019, -0.273146072229, 0.236871183868, 0, pi/2, 0),
         (0.0572620884946, -0.471084131091, 0.257903652202, 0, pi/2, 0)],
        [(-0.165670683854, -0.27325092677, 0.234959369791, 0, pi/2, 0),
         (0.052343375325, -0.438820609084, 0.258540150491, 0, pi/2, 0)],
        [(-0.11724126734, -0.272629250978, 0.235591521231, 0, pi/2, 0),
         (-0.30547478956, -0.469060379203, 0.259206533778, 0, pi/2, 0)],
        [(-0.0690097487039, -0.271989014353, 0.233774723158, 0, pi/2, 0),
         (-0.304454179627, -0.437760065, 0.256487089268, 0, pi/2, 0)] ]
    
    rospy.logwarn("Moving to initial pose...")
    assembler.move_to_init_pos()

    for idx in range(len(points)):
        rospy.logwarn("Moving pin #%d..." % idx)
        transfer_pin(assembler, points[idx][0], points[idx][1])

    rospy.logwarn("Moving to initial pose...")
    assembler.move_to_init_pos()

    rospy.logwarn("Mission complete!")