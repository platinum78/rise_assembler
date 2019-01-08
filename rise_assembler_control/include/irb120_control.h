#include <iostream>
#include <math.h>
#include <vector>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <Eigen/Dense>
#include "assembler_params.h"
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
    Eigen::VectorXd d;
    Eigen::VectorXd a;
    Eigen::VectorXd theta;
    Eigen::VectorXd theta_target;
    Eigen::VectorXd configuration;
    Eigen::VectorXd configuration_target;
    Eigen::VectorXd alpha;
    Eigen::VectorXd P;
    Eigen::VectorXd P_endeffector;
    Eigen::MatrixXd R;
    Eigen::MatrixXd R_joint;
    Eigen::MatrixXd T;
    Eigen::MatrixXd T_joint;
    Eigen::MatrixXd T_endeffector;
    Eigen::MatrixXd configurationPath;
    
    // Path storages
    Eigen::MatrixXd jointTrajectory;
    Eigen::VectorXd timesteps;

    // ROS publisher, subscriber, service server, and message
    ros::NodeHandle n;
    ros::Publisher joint_trajectory_publisher;
    ros::Publisher motion_status_publisher;
    ros::Subscriber joint_states_subscriber;
    ros::Subscriber irb120_move_subscriber;
    trajectory_msgs::JointTrajectory message;

  public:
    IRB120_Controller(ros::NodeHandle handle);

    // Calculation functions
    Eigen::MatrixXd getTransMatrix(double, double, double, double);
    Eigen::MatrixXd getHomoTransMatrix(Eigen::MatrixXd, Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd);
    Eigen::MatrixXd getR03(Eigen::VectorXd);
    void forwardKinematicsSolver(Eigen::VectorXd);
    void inverseKinematicsSolver(Eigen::MatrixXd);
    Eigen::MatrixXd pathGen(int path_mode, int path_steps);
    Eigen::VectorXd timeProfileMaker(double travel_time, int steps, double angle=60);
    Eigen::MatrixXd jointTrajectoryGen(Eigen::MatrixXd configurationPath, int path_mode, double travel_time);
    void publishTrajectory();
    
    // Utility functions
    
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
    d = Eigen::VectorXd(6); d << 290, 0, 0, 302, 0, 72;
    a = Eigen::VectorXd(6); a << 0, 270, 70, 0, 0, 0;
    alpha = Eigen::VectorXd(6); alpha << 90, 0, 90, -90, 90, 0;
    alpha *= (M_PI/180);        // Convert from degrees to radians
    
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

// Get homogeneous transformation for single joint
Eigen::MatrixXd IRB120_Controller::getTransMatrix(double theta, double d_, double a_, double alpha_)
{
    Eigen::MatrixXd T01(4, 4);
    T01 <<  cos(theta),     -sin(theta) * cos(alpha_),  sin(theta) * sin(alpha_),   a_ * cos(theta),
            sin(theta),     cos(theta) * cos(alpha_),   -cos(theta) * sin(alpha_),  a_ * sin(theta),
            0,              sin(alpha_),                cos(alpha_),                d_,
            0,              0,                          0,                          1;
    return T01;
}

// Get homogeneous transformation matrix for end effector
Eigen::MatrixXd IRB120_Controller::getHomoTransMatrix(Eigen::MatrixXd theta, Eigen::VectorXd d, Eigen::VectorXd a, Eigen::VectorXd alpha)
{
    Eigen::MatrixXd T01 = getTransMatrix(theta(0), d(0), a(0), alpha(0));
    Eigen::MatrixXd T12 = getTransMatrix(theta(1), d(1), a(1), alpha(1));
    Eigen::MatrixXd T23 = getTransMatrix(theta(2), d(2), a(2), alpha(2));
    Eigen::MatrixXd T34 = getTransMatrix(theta(3), d(3), a(3), alpha(3));
    Eigen::MatrixXd T45 = getTransMatrix(theta(4), d(4), a(4), alpha(4));
    Eigen::MatrixXd T56 = getTransMatrix(theta(5), d(5), a(5), alpha(5));
    Eigen::MatrixXd T06 = T01 * T12 * T23 * T34 * T45 * T56;
    
    return T06;
}

Eigen::MatrixXd IRB120_Controller::getR03(Eigen::VectorXd theta_)
{
    Eigen::MatrixXd R03(3, 3);
    Eigen::MatrixXd T03;
    Eigen::MatrixXd T01;
    Eigen::MatrixXd T12;
    Eigen::MatrixXd T23;

    T01 = getTransMatrix(theta_(0), d(0), a(0), alpha(0));
    T12 = getTransMatrix(theta_(1), d(1), a(1), alpha(1));
    T23 = getTransMatrix(theta_(2), d(2), a(2), alpha(2));
    T03 = T01 * T12 * T23;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            R03(i, j) = T03(i, j);
    return R03;
}

// Forward kinematics solver
void IRB120_Controller::forwardKinematicsSolver(Eigen::VectorXd theta_)
{
    theta = theta_;
    theta(0) = theta(0) * (M_PI / 180);
    theta(1) = (theta(1) + 90) * (M_PI / 180);
    theta(2) = theta(2) * (M_PI / 180);
    theta(3) = theta(3) * (M_PI / 180);
    theta(4) = theta(4) * (M_PI / 180);
    theta(5) = theta(5) * (M_PI / 180);
}

// Inverse kinematics solver
void IRB120_Controller::inverseKinematicsSolver(Eigen::MatrixXd H)
{  
    int i, j, index;
    Eigen::MatrixXd Pos(3, 1);
    Eigen::MatrixXd Rot(3, 3);
    Eigen::MatrixXd k(3, 1);
    Eigen::MatrixXd Pos1(3, 1);
    Eigen::MatrixXd JointAngles(1, 6);
    Eigen::MatrixXd R03(3, 3);
    Eigen::MatrixXd R36(3, 3);
    Eigen::MatrixXd E_Pos(3, 1);
    double x, y, z, R, alpha, beta, theta1, theta2, theta3, theta4, theta5, theta6, C2, S2, temp;
    for (i = 0; i <= 2; i++)
    {
        Pos(i, 0) = H(i, 3);
        if (i == 2)
            k(i, 0) = 1;
        else
            k(i, 0) = 0;
        for (j = 0; j <= 2; j++)
            Rot(i, j) = H(i, j);
    }

    Eigen::MatrixXd b(3, 1);
    b(0, 0) = 0;
    b(1, 0) = 1;
    b(2, 0) = 1;
    Eigen::MatrixXd end(3, 1);
    // E_Pos = Rot*b;
    Pos(0, 0) = Pos(0, 0);
    Pos(1, 0) = Pos(1, 0) - (32.5);
    Pos(2, 0) = Pos(2, 0) + (165);

    Pos1 = Pos - (72) * Rot * k;
    x = Pos1(0, 0);
    y = Pos1(1, 0);
    z = Pos1(2, 0);

    theta1 = atan2(y, x);
    R = sqrt(x * x + y * y);
    alpha = atan2(70, 302);
    beta = atan2(R, z - 290);
    C2 = (pow(R, 2) + pow(z - 290, 2) + pow(270, 2) - pow(302, 2) - pow(70, 2)) / (540 * sqrt(pow(R, 2) + pow(z - 290, 2)));
    S2 = sqrt(1 - pow(C2, 2));

    theta2 = atan2(S2, C2) - beta;

    temp = atan2(z - 290 - 270 * cos(theta2), R + 270 * sin(theta2));
    theta3 = temp - theta2 - alpha;

    Eigen::VectorXd t(3);
    t(0) = theta1;
    t(1) = theta2 + (90 * M_PI / 180.0);
    t(2) = theta3;
    R03 = getR03(t);
    R36 = R03.transpose() * Rot;
    double S5 = sqrt(pow(R36(0, 2), 2) + pow(R36(1, 2), 2));
    theta5 = atan2(S5, R36(2, 2));

    theta4 = atan2(R36(1, 2) / S5, R36(0, 2) / S5);

    theta6 = atan2(R36(2, 1) / S5, -R36(2, 0) / S5);

    theta_target(0) = theta1;
    theta_target(1) = theta2;
    theta_target(2) = theta3;
    theta_target(3) = theta4;
    theta_target(4) = theta5;
    theta_target(5) = theta6;
}

// End-effector path generator
Eigen::MatrixXd IRB120_Controller::pathGen(int path_mode, int path_steps)
{
    Eigen::MatrixXd configurationDiff = configuration_target - configuration;
    
    if (path_mode == PATH_LINEAR)
    {
        // Make path through linear interpolation
        configurationPath = Eigen::MatrixXd(path_steps+1, 6);
        for (int idx = 0; idx <= path_steps; idx++)
            for (int joint = 0; joint < 6; joint++)
                configurationPath(idx, joint) = configurationDiff(joint) / path_steps * idx;
    }
    else
    {
        // Make path with only starting and ending point
        configurationPath = Eigen::MatrixXd(1, 6);
        for (int joint = 0; joint < 6; joint++)
            configurationPath(0, joint) = configuration_target(joint);
    }
    
    return configurationPath;
}

Eigen::VectorXd IRB120_Controller::timeProfileMaker(double travel_time, int steps, double angle)
{
    // Initialization
    double theta = angle * M_PI / 180;
    double L = travel_time;
    timesteps = Eigen::VectorXd(steps + 1);
    
    // Calculate parameters
    double radius = (sin(theta) - cos(theta)) / (1 - cos(theta)) / 2 * L;
    double departure = radius * (1 - cos(theta));
    double arrival = L - radius * (1 - cos(theta));

    // Make profile
    double x = 0;
    for (int idx = 0; idx < steps + 1; idx++)
    {
        x = travel_time / steps * idx;
        if (x < departure)
            timesteps(idx) = sqrt(2 * radius * x - pow(x,2));
        else if (x < arrival)
            timesteps(idx) = ((x - radius) * cos(theta) + radius) / sin(theta);
        else
            timesteps(idx) = L - sqrt(2 * radius * (L - x) - pow(x-L,2));       
    }

    return timesteps;
}

Eigen::MatrixXd IRB120_Controller::jointTrajectoryGen(Eigen::MatrixXd configurationPath, int path_mode, double travel_time)
{
    Eigen::VectorXd thetaDiff = (theta_target - theta).cwiseAbs();
    double maxOffset = thetaDiff.maxCoeff();
    int pathPoints = configurationPath.rows();
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