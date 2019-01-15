import sys, copy, rospy, moveit_commander, moveit_msgs.msg, geometry_msgs.msg
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
        self.get_joint_angles()

    def move_to_joint_angle(self, pos1, pos2, pos3, pos4, pos5, pos6):
        # Move to non-singularity position
        self.joint_goal = copy.deepcopy(self.joint_angles)
        self.joint_goal[:6] = [pos1, pos2, pos3, pos4, pos5, pos6]
        angle_string = "%6d, %6d, %6d, %6d, %6d, %6d" % \
                        (pos1, pos2, pos3, pos4, pos5, pos6)
        rospy.loginfo("Joint position goals: " + angle_string)
        
        # Wait for redundant movements to complete.
        rospy.loginfo("Waiting for redundant movements to be completed...")
        self.abb_irb120.go(wait=True)
        self.abb_irb120.stop()
        rospy.loginfo("Redundant movements complete. Now start movement by path.")
        self.abb_irb129.clear_pose_targets()
        self.abb_irb120.go(self.joint_goal, wait=True)
        rospy.sleep(2)
        # self.abb_irb120.stop()
        rospy.loginfo("Finished movement.")

    def move_by_path(self, x, y, z, r, p, y_, path_resolution=0.01):
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
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
        rospy.loginfo("Computing path.")
        (trajectory_plan, fraction) = abb_irb120.compute_cartesian_path(self.waypoints, path_resolution, 0.0)

        trajectory_plan.joint_trajectory.points.pop(0)
        sleep_time = trajectory_plan.joint_trajectory.points[-1].time_from_start.secs \
                     + trajectory_plan.joint_trajectory.points[-1].time_from_start.nsecs / 1e9 + 1
        
        rospy.loginfo("Waiting for redundant movements to be completed...")
        self.abb_irb120.go(wait=True)
        self.abb_irb120.stop()
        rospy.loginfo("Redundant movements complete. Now start movement by path.")
        self.abb_irb120.execute(trajectory_plan, wait=True)
        # rospy.sleep(sleep_time)
    
    def get_joint_angles(self, log=True):
        self.joint_angles = self.abb_irb120.get_current_joint_values()
        if log:    
            angles_string = "%6d, %6d, %6d, %6d, %6d, %6d" % \
                            tuple(self.joint_angles[x] for x in range(6))
            rospy.loginfo("Current joint angles: " + angles_string)
    

if __name__ == "__main__":
    rospy.init_node("irb120_control_node", anonymous=True)
    rospy.