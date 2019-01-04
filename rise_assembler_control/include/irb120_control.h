#include <iostream>
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include "./Eigen/Dense"
// #include <Eigen/Dense>
#include <iomanip>

#define PI 3.14159265358979323846
#define RAD2DEG(X) (X*180/PI)
#define DEG2RAD(X) (X*PI/180)

using namespace Eigen;
using namespace std;

typedef struct PathPoint_
{
    struct PathPoint_ *next;
    VectorXd configuration;
} PathPoint;

typedef struct TaskList_
{
    PathPoint *head;
    PathPoint *tail;

} TaskList;

class IRB120_Controller
{
  private:
    // Variables for control system
    VectorXd d;
    VectorXd a;
    VectorXd theta;
    VectorXd theta_target;
    VectorXd alpha;
    VectorXd endEffectorPos;
    MatrixXd jointRotationMatrix;
    MatrixXd jointTransMatrix;
    MatrixXd endEffectorTransMatrix;

    // ROS topic publisher
    ros::Publisher joint_01_pub;
    ros::Publisher joint_02_pub;
    ros::Publisher joint_03_pub;
    ros::Publisher joint_04_pub;
    ros::Publisher joint_05_pub;
    ros::Publisher joint_06_pub;
    
    // ROS service server
    ros::ServiceServer move_robot_service;

    // ROS message for each joint position
    std_msgs::Float64 joint_01_msg;
    std_msgs::Float64 joint_02_msg;
    std_msgs::Float64 joint_03_msg;
    std_msgs::Float64 joint_04_msg;
    std_msgs::Float64 joint_05_msg;
    std_msgs::Float64 joint_06_msg;

  public:
    IRB120_Controller(ros::NodeHandle n);

    // Calculation functions
    MatrixXd getTransMatrix(double, double, double, double);
    MatrixXd getHomoTransMatrix(MatrixXd, VectorXd, VectorXd, VectorXd);
    MatrixXd getR03(VectorXd);
    void forwardKinematics(VectorXd);
    void inverseKinematics(MatrixXd);
    MatrixXd pathGen();
    MatrixXd path2Profile();
    void publishPosition();
    bool moveRobot();
    
    // Utility functions
};