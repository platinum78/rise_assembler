#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <math.h>
#include <vector>
#include <string>
// using namespace trajectory_msgs;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "joint_trajectory_test");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<trajectory_msgs::JointTrajectory>("/joint_path_command", 1000);
    ros::Rate loop_rate(10);

    double sine[100]; double cosine[100];
    for (int idx = 0; idx < 100; idx++)
    {
        sine[idx] = sin(2 * M_PI * idx / 100);
        cosine[idx] = cos(2 * M_PI * idx / 100);
    }

    trajectory_msgs::JointTrajectory message;
    message.header.stamp.sec = 0;
    message.header.stamp.nsec = 0;
    message.header.frame_id = "/base_frame";
    message.joint_names.resize(6);
    message.joint_names[0] = "joint_1";
    message.joint_names[1] = "joint_2";
    message.joint_names[2] = "joint_3";
    message.joint_names[3] = "joint_4";
    message.joint_names[4] = "joint_5";
    message.joint_names[5] = "joint_6";
    std::vector<trajectory_msgs::JointTrajectoryPoint> point(1);
    message.points.resize(1);
    message.points[0].positions.resize(6);
    
    /*
    for (int idx = 0; idx < 100; idx++)
    {
        message.points[0].positions[0] = cosine[idx];
        message.points[0].positions[1] = 0;
        message.points[0].positions[2] = sine[idx];
        message.points[0].positions[3] = 0;
        message.points[0].positions[4] = 0;
        message.points[0].positions[5] = 0;
        message.points[0].time_from_start.sec = 0;
        message.points[0].time_from_start.nsec = 100000000;
        pub.publish(message);
        loop_rate.sleep();
    }
    */
    message.points[0].positions[0] = 0;
    message.points[0].positions[1] = 0;
    message.points[0].positions[2] = 0;
    message.points[0].positions[3] = 0;
    message.points[0].positions[4] = 0;
    message.points[0].positions[5] = 0;
    message.points[0].time_from_start.sec = 1;
    message.points[0].time_from_start.nsec = 0;
    pub.publish(message);
    ros::Duration(1).sleep();

    
}