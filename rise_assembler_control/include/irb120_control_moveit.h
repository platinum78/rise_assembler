#include <iostream>
#include <cmath>
#include <vector>
#include <string>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <Eigen/Dense>
#include "./assembler_params.h"
#include "rise_assembler_control/IRB120_move.h"
// #include "rise_assembler_control/IRB120_move.h"
#define RAD2DEG(X) (X*180/M_PI)
#define DEG2RAD(X) (X*M_PI/180)

////////////////////////////////////////////////////////////////////////////////
// CLASS MEMBERS MANIFESTATION
////////////////////////////////////////////////////////////////////////////////

class IRB120_Controller
{
  private:
    // Variables for control system
    
    
    // Path storages
    Eigen::MatrixXd jointTrajectory;
    Eigen::VectorXd timesteps;
    std::string PLANNING_GROUP;

    // ROS publisher, subscriber, service server, and message
    ros::NodeHandle n;
    ros::Publisher joint_trajectory_publisher;
    ros::Publisher motion_status_publisher;
    ros::Subscriber joint_states_subscriber;
    ros::Subscriber irb120_move_subscriber;
    trajectory_msgs::JointTrajectory message;

  public:
    IRB120_Controller(ros::NodeHandle handle);

    void pathGen();

    // Utility functions
    void startControlLoop();
    void printTheta();
    void copySinglePoint()
    {
        message.points.resize(1);
        message.points[0].positions.resize(6);
        message.points[0].time_from_start.sec = 2;
        message.points[0].time_from_start.nsec = 0;
        for (int idx = 0; idx < 6; idx++)
            message.points[0].positions[idx] = theta_target(idx);
        ros::Duration(0.5).sleep();
        joint_trajectory_publisher.publish(message);
        ros::Duration(2.0).sleep();
    }
    
    // Callback functions
    void irb120MoveCallback(const rise_assembler_control::IRB120_move &msg);
    void updateJointStatesCallback(const sensor_msgs::JointState&);
};


////////////////////////////////////////////////////////////////////////////////
// CLASS MEMBER DEFINITION
////////////////////////////////////////////////////////////////////////////////

IRB120_Controller::IRB120_Controller(ros::NodeHandle handle)
{
    // Static parameters
    n = handle;
    PLANNING_GROUP << "abb_irb120";
    moveit::planning
    
    // Dynamic parameters
    theta = Eigen::VectorXd(6);
    theta_target = Eigen::VectorXd(6);
    P = Eigen::VectorXd(6);
    P_endeffector = Eigen::VectorXd(6);
    R = Eigen::MatrixXd(3, 3);
    R_joint = Eigen::MatrixXd(3, 3);
    T = Eigen::MatrixXd(4, 4);
    T_joint = Eigen::MatrixXd(4, 4);
    T_endeffector = Eigen::MatrixXd(4, 4);

    // ROS publishers, messages
    joint_trajectory_publisher = n.advertise<trajectory_msgs::JointTrajectory>("/joint_path_command", 1000);
    irb120_move_subscriber = n.subscribe("/assembler_move/irb120/command", 100, &IRB120_Controller::irb120MoveCallback, this);
    joint_states_subscriber = n.subscribe("/joint_states", 100, &IRB120_Controller::updateJointStatesCallback, this);
    message.header.stamp.sec = 0; message.header.stamp.nsec = 0; message.header.frame_id = "/base_frame";
    message.joint_names.resize(6);
    message.joint_names[0] = "joint_1"; message.joint_names[1] = "joint_2";
    message.joint_names[2] = "joint_3"; message.joint_names[3] = "joint_4";
    message.joint_names[4] = "joint_5"; message.joint_names[5] = "joint_6";
}



void IRB120_Controller::publishTrajectory()
{
    // Read step count info and resize message length
    int steps = timesteps.size() - 1;
    message.points.resize(steps + 1);

    // Put each point into message structure
    for (int idx = 0; idx < steps + 1; idx++)
    {
        message.points[idx].positions.resize(6);
        message.points[idx].positions[0] = jointTrajectory(idx, 0);
        message.points[idx].positions[1] = jointTrajectory(idx, 1);
        message.points[idx].positions[2] = jointTrajectory(idx, 2);
        message.points[idx].positions[3] = jointTrajectory(idx, 3);
        message.points[idx].positions[4] = jointTrajectory(idx, 4);
        message.points[idx].positions[5] = jointTrajectory(idx, 5);
        message.points[idx].time_from_start = ros::Duration(timesteps(idx));
    }

    // Publish message and wait until movement finishes
    double travelTime = timesteps.maxCoeff();
    joint_trajectory_publisher.publish(message);
    ros::Duration(travelTime).sleep();
}

void IRB120_Controller::startControlLoop()
{
    ros::spin();
}

void IRB120_Controller::printTheta()
{
    ROS_INFO("%f, %f, %f, %f, %f, %f", theta_target(0), theta_target(1),
                                        theta_target(2), theta_target(3),
                                        theta_target(4), theta_target(5));
}

void IRB120_Controller::irb120MoveCallback(const rise_assembler_control::IRB120_move &msg)
{
    ROS_INFO("Move callback called.");

    // return true;
}

void IRB120_Controller::updateJointStatesCallback(const sensor_msgs::JointState& message)
{
    theta(0) = message.position[0];
    theta(1) = message.position[1];
    theta(2) = message.position[2];
    theta(3) = message.position[3];
    theta(4) = message.position[4];
    theta(5) = message.position[5];
}