#!/usr/bin/python

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import std_msgs.msg
import numpy as np

def publisher():
    rospy.init_node("joint_trajectory_test")
    pub = rospy.Publisher("/joint_path_command", JointTrajectory, queue_size=1000)
    rate = rospy.Rate(10)

    segment_cnt = 100
    x = np.linspace(0, 2*np.pi, segment_cnt)
    sine = np.sin(x)
    cosine = np.cos(x)

    message = JointTrajectory()
    message.header.stamp.secs = 0
    message.header.stamp.nsecs = 0
    message.header.frame_id = "/base_frame"
    message.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
    message.points.append(JointTrajectoryPoint())

    for idx in range(segment_cnt):
        # point = JointTrajectoryPoint()
        message.points[0].positions = [cosine[idx], 0, sine[idx], 0, 0, 0]
        # point.velocities = [0.01, 0.01, 0.01, 0.01, 0.01, 0.01]
        message.points[0].time_from_start.secs = 0
        message.points[0].time_from_start.nsecs = 100000000
        # message.points.append(point)
        pub.publish(message)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass