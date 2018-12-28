#include "../include/IRB120_Commander.h"

// IRB120_Commander class initializer
IRB120_Commander::IRB120_Commander(ros::NodeHandle n)
{
    // Initialize and advertise each publisher
    joint_01_pub = n.advertise<std_msgs::Float64>("/assembler/irb120/joint_position_controller/01/command", 1000);
    joint_02_pub = n.advertise<std_msgs::Float64>("/assembler/irb120/joint_position_controller/02/command", 1000);
    joint_03_pub = n.advertise<std_msgs::Float64>("/assembler/irb120/joint_position_controller/03/command", 1000);
    joint_04_pub = n.advertise<std_msgs::Float64>("/assembler/irb120/joint_position_controller/04/command", 1000);
    joint_05_pub = n.advertise<std_msgs::Float64>("/assembler/irb120/joint_position_controller/05/command", 1000);
    joint_06_pub = n.advertise<std_msgs::Float64>("/assembler/irb120/joint_position_controller/06/command", 1000);
    gripper_pub = n.advertise<std_msgs::Float64>("/assembler/gripper/joint_position_controller/command", 1000);
}

// Forward kinematics solver
VectorXd ForwardKinematics()
{

}

// Inverse kinematics solver
VectorXd InverseKinematics()
{

}

// End-effector path generator
MatrixXd PathGen()
{

}

// Convert end-effector path to joint profile:
// start and stop at same time
MatrixXd Path2Profile_Simultaneous()
{

}

// Convert end-effector path to joint profile:
// each link start and stop at different time
MatrixXd Path2Profile_Separate()
{

}