import sys, copy, rospy, moveit_commander, moveit_msgs.msg, geometry_msgs.msg, time
from math import pi, sqrt, sin, cos, tan
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler
import numpy as np

# This message will be copied for every instances: mimicry of C++ struct
class grasp_points_msg:
    def __init__(self, dataset_arr=None):
        # "approach_angle" should be given in [longitude, latitude] form.
        self.start = {
            "position": [0, 0, 0],
            "orientation": [0, 0, 0],
            "approach_dist": 0.0,
            "approach_angle": [0, 0],
            "withdraw_dist": 0.0,
            "withdraw_angle": [0, 0]
        }
        self.end = {
            "position": [0, 0, 0],
            "orientation": [0, 0, 0],
            "approach_dist": 0.0,
            "approach_angle": [0, 0],
            "withdraw_dist": 0.0,
            "withdraw_angle": [0, 0]
        }

        if dataset_arr is not None:
            self.set_from_dataset(dataset_arr)
    
    def set_start_point(self, pose, approach_offset, withdraw_offset):
        self.start["position"] = pose[0:3]
        self.start["orientation"] = pose[3:6]
        self.start["approach_dist"] = approach_offset[0]
        self.start["approach_angle"] = [approach_offset[1], approach_offset[2]]
        self.start["withdraw_dist"] = withdraw_offset[0]
        self.start["withdraw_angle"] = [withdraw_offset[1], withdraw_offset[2]]
    
    def set_end_point(self, pose, approach_offset, withdraw_offset):
        self.end["position"] = pose[0:3]
        self.end["orientation"] = pose[3:6]
        self.end["approach_dist"] = approach_offset[0]
        self.end["approach_angle"] = [approach_offset[1], approach_offset[2]]
        self.end["withdraw_dist"] = withdraw_offset[0]
        self.end["withdraw_angle"] = [withdraw_offset[1], withdraw_offset[2]]
    
    def set_from_dataset(self, dataset_arr):
        # Points are given in the form of [start_pose, start_approach_offset, end_pose, end_approach_offset].
        # Each pose is given in the form of (x, y, z, r, p, y_)
        # Each offset is given in (distance, londitude, latitude).
        self.set_start_point(dataset_arr[0], dataset_arr[1], dataset_arr[2])
        self.set_end_point(dataset_arr[3], dataset_arr[4], dataset_arr[5])


# Improvemenwets on grasp_and_move()
def grasp_and_move(assembler, points_msg):
    # Get starting and ending point location.
    start = points_msg.start["position"] + points_msg.start["orientation"]
    end = points_msg.end["position"] + points_msg.end["orientation"]
    # end[3] += assembler.joint6_compensation

    # Calculate the approaching point of the starting point from the given angles and distances.
    r = points_msg.start["approach_dist"]
    theta, rho = points_msg.start["approach_angle"][0], points_msg.start["approach_angle"][1]
    print "r: ", r, "theta: ", theta, "rho: ", rho
    print "offset: ", r * cos(rho) * cos(theta), r * cos(rho) * sin(theta), r * sin(rho)
    start_approach = copy.deepcopy(start)
    start_approach[0] += r * cos(rho) * cos(theta)
    start_approach[1] += r * cos(rho) * sin(theta)
    start_approach[2] += r * sin(rho)

    r = points_msg.start["withdraw_dist"]
    theta, rho = points_msg.start["withdraw_angle"][0], points_msg.start["withdraw_angle"][1]
    print "r: ", r, "theta: ", theta, "rho: ", rho
    print "offset: ", r * cos(rho) * cos(theta), r * cos(rho) * sin(theta), r * sin(rho)
    start_withdraw = copy.deepcopy(start)
    start_withdraw[0] += r * cos(rho) * cos(theta)
    start_withdraw[1] += r * cos(rho) * sin(theta)
    start_withdraw[2] += r * sin(rho)

    # Calculate the approaching points of the ending point from the given angles and distances.
    r = points_msg.end["approach_dist"]
    theta, rho = points_msg.end["approach_angle"][0], points_msg.end["approach_angle"][1]
    print "offset: ", r * cos(rho) * cos(theta), r * cos(rho) * sin(theta), r * sin(rho)
    end_approach = copy.deepcopy(end)
    end_approach[0] += r * cos(rho) * cos(theta)
    end_approach[1] += r * cos(rho) * sin(theta)
    end_approach[2] += r * sin(rho)

    r = points_msg.end["withdraw_dist"]
    theta, rho = points_msg.end["withdraw_angle"][0], points_msg.end["withdraw_angle"][1]
    print "offset: ", r * cos(rho) * cos(theta), r * cos(rho) * sin(theta), r * sin(rho)
    end_withdraw = copy.deepcopy(end)
    end_withdraw[0] += r * cos(rho) * cos(theta)
    end_withdraw[1] += r * cos(rho) * sin(theta)
    end_withdraw[2] += r * sin(rho)

    rospy.logwarn("Moving to pickup position...")
    assembler.move_to_pose(start_approach)
        
    rospy.logwarn("Opening gripper...")
    assembler.gripper_open()

    rospy.logwarn("Descending to pick part...")
    assembler.move_by_cartesian_path(start, path_resolution=0.005)
        
    rospy.logwarn("Closing gripper...")
    assembler.gripper_grip()
        
    rospy.logwarn("Picked part up. Now ascending...")
    assembler.move_by_cartesian_path(start_withdraw, path_resolution=0.005)
        
    rospy.logwarn("Moving to desntination...")
    assembler.move_to_pose(end_approach)
        
    rospy.logwarn("Arrived to destination. Pulling part into slot...")
    assembler.move_by_cartesian_path(end, path_resolution=0.005)
        
    rospy.logwarn("Opening gripper...")
    assembler.gripper_open()
        
    rospy.logwarn("Pulled part into slot. Now withdrawing from workplace...")
    assembler.move_by_cartesian_path(end_withdraw, path_resolution=0.005)

    rospy.logwarn("Closing gripper...")
    # assembler.gripper_grip()

    rospy.logwarn("Cycle complete.")

