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
#include "./Eigen/Dense"
// #include <Eigen/Dense>
#include <iomanip>

#include "rise_assembler_control/IRB120_move.h"

#define M_PI 3.14159265358979323846
#define RAD2DEG(X) (X*180/M_PI)
#define DEG2RAD(X) (X*M_PI/180)

using namespace Eigen;
using namespace std;

////////////////////////////////////////////////////////////////////////////////
// CLASS MEMBERS MANIFESTATION
////////////////////////////////////////////////////////////////////////////////

class IRB120_Controller
{
  private:
    // Variables for control system
    VectorXd d;
    VectorXd a;
    VectorXd theta;
    VectorXd theta_target;
    VectorXd alpha;
    VectorXd P;
    VectorXd P_endeffector;
    MatrixXd R;
    MatrixXd R_joint;
    MatrixXd T;
    MatrixXd T_joint;
    MatrixXd T_endeffector;
    
    // Path storages
    MatrixXd jointTrajectory;
    VectorXd timesteps;

    // ROS publisher, subscriber, service server, and message
    ros::Publisher joint_trajectory_publisher;
    ros::Subscriber joint_states_subscriber;
    ros::ServiceServer irb120_move_service;
    trajectory_msgs::JointTrajectory message;

  public:
    IRB120_Controller(ros::NodeHandle n);

    // Calculation functions
    MatrixXd getTransMatrix(double, double, double, double);
    MatrixXd getHomoTransMatrix(MatrixXd, VectorXd, VectorXd, VectorXd);
    MatrixXd getR03(VectorXd);
    void forwardKinematicsSolver(VectorXd);
    void inverseKinematicsSolver(MatrixXd);
    MatrixXd pathGen();
    MatrixXd path2Profile();
    void publishTrajectory();
    bool moveRobot();
    
    // Utility functions
    
    // Callback functions
    void updateJointStates(sensor_msgs::JointState&);
    void irb120MoveCallback()
};


////////////////////////////////////////////////////////////////////////////////
// CLASS MEMBER DEFINITION
////////////////////////////////////////////////////////////////////////////////

IRB120_Controller::IRB120_Controller(ros::NodeHandle n)
{
    ////////////////////////////////////////////////////////////////////////////////
    // Initialize parameters and input robot dimensions
    ////////////////////////////////////////////////////////////////////////////////
    
    // Static parameters
    d = VectorXd(6); d << 290, 0, 0, 302, 0, 72;
    a = VectorXd(6); a << 0, 270, 70, 0, 0, 0;
    alpha = VectorXd(6); alpha << 90, 0, 90, -90, 90, 0;
    alpha *= (M_PI/180);        // Convert from degrees to radians
    
    // Dynamic parameters
    theta = VectorXd(6);
    theta_target = VectorXd(6);
    P = VectorXd(6;)
    P_endeffector = VectorXd(6);
    R = MatrixXd(3, 3);
    R_joint = MatrixXd(3, 3);
    T = MatrixXd(4, 4);
    T_joint = MatrixXd(4, 4);
    T_endeffector = MatrixXd(4, 4);


    ////////////////////////////////////////////////////////////////////////////////
    // Initialize ROS communications
    ////////////////////////////////////////////////////////////////////////////////
    
    // ROS publishers, messages
    joint_trajectory_publisher = n.advertise<trajectory_msgs::JointTrajectory>("/joint_path_command", 1000);
    move_robot_service = n.advertiseService("/irb120_move", irb120MoveCallback);
    joint_states_subscriber = n.subscribe("/joint_states", 1000, updateJointStates);
    message.header.stamp.sec = 0; message.header.stamp.nsec = 0; message.header.frame_id = "/base_frame";
    message.joint_names.resize(6);
    message.joint_names[0] = "joint_1"; message.joint_names[1] = "joint_2";
    message.joint_names[2] = "joint_3"; message.joint_names[3] = "joint_4";
    message.joint_names[4] = "joint_5"; message.joint_names[5] = "joint_6";
}

// Get homogeneous transformation for single joint
MatrixXd IRB120_Controller::getTransMatrix(double theta, double d_, double a_, double alpha_)
{
    MatrixXd T01(4, 4);
    T01 <<  cos(theta),     -sin(theta) * cos(alpha_),  sin(theta) * sin(alpha_),   a_ * cos(theta),
            sin(theta),     cos(theta) * cos(alpha_),   -cos(theta) * sin(alpha_),  a_ * sin(theta),
            0,              sin(alpha_),                cos(alpha_),                d_,
            0,              0,                          0,                          1;
    return T01;
}

void IRB120_Controller::irb120MoveCallback()
{

}

// Get homogeneous transformation matrix for end effector
MatrixXd IRB120_Controller::getHomoTransMatrix(MatrixXd theta, VectorXd d, VectorXd a, VectorXd alpha)
{
    MatrixXd T01 = getTransMatrix(theta(0), d(0), a(0), alpha(0));
    MatrixXd T12 = getTransMatrix(theta(1), d(1), a(1), alpha(1));
    MatrixXd T23 = getTransMatrix(theta(2), d(2), a(2), alpha(2));
    MatrixXd T34 = getTransMatrix(theta(3), d(3), a(3), alpha(3));
    MatrixXd T45 = getTransMatrix(theta(4), d(4), a(4), alpha(4));
    MatrixXd T56 = getTransMatrix(theta(5), d(5), a(5), alpha(5));
    MatrixXd T06 = T01 * T12 * T23 * T34 * T45 * T56;
    
    return T06;
}

MatrixXd IRB120_Controller::getR03(VectorXd theta_)
{
    MatrixXd R03(3, 3);
    MatrixXd T03;
    MatrixXd T01;
    MatrixXd T12;
    MatrixXd T23;

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
void IRB120_Controller::forwardKinematicsSolver(VectorXd theta_)
{
    theta = theta_;
    theta(0) = theta(0) * (M_PI / 180);
    theta(1) = (theta(1) + 90) * (M_PI / 180);
    theta(2) = theta(2) * (M_PI / 180);
    theta(3) = theta(3) * (M_PI / 180);
    theta(4) = theta(4) * (M_PI / 180);
    theta(5) = theta(5) * (PI / 180);
}

// Inverse kinematics solver
void IRB120_Controller::inverseKinematicsSolver(MatrixXd H)
{  
    int i, j, index;
    MatrixXd Pos(3, 1);
    MatrixXd Rot(3, 3);
    MatrixXd k(3, 1);
    MatrixXd Pos1(3, 1);
    MatrixXd JointAngles(1, 6);
    MatrixXd R03(3, 3);
    MatrixXd R36(3, 3);
    MatrixXd E_Pos(3, 1);
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

    MatrixXd b(3, 1);
    b(0, 0) = 0;
    b(1, 0) = 1;
    b(2, 0) = 1;
    MatrixXd end(3, 1);
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

    VectorXd t(3);
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
MatrixXd IRB120_Controller::pathGen()
{
    VectorXd 
}

// Convert end-effector path to joint profile:
// start and stop at same time
MatrixXd IRB120_Controller::path2Profile()
{

}

void IRB120_Controller::publishTrajectory()
{
    // Create ROS messages

    // Publish each message
    joint_trajectory_publisher.publish(message);
}

bool IRB120_Controller::moveRobot()
{
    
}

void IRB120_Controller::updateJointStates(sensor_msgs::JointState& message)
{
    theta(0) = message.position[0];
    theta(1) = message.position[1];
    theta(2) = message.position[2];
    theta(3) = message.position[3];
    theta(4) = message.position[4];
    theta(5) = message.position[5];
}