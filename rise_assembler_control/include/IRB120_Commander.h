#include <iostream>
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include "./Eigen/Dense"
// #include <Eigen/Dense>
#include <iomanip>
#include <stdexcept>

#define TASK_TYPE_MOVE 0
#define TASK_TYPE_GRIP 1
#define TASK_TYPE_OPEN 2

using namespace Eigen;
using namespace std;

typedef struct PathPoint_
{
    struct PathPoint_* next;
    VectorXd configuration;
} PathPoint;

typedef struct TaskList_
{
    PathPoint* head;
    PathPoint* tail;
    
} TaskList;

class IRB120_Commander
{
private:
    // Variables for control system
    VectorXd armLength;
    VectorXd jointAngleStatus;
    VectorXd jointAngleTarget;
    VectorXd endEffectorPos;
    MatrixXd rotationMatrix;

    // ROS topic publisher
    ros::Publisher joint_01_pub;
    ros::Publisher joint_02_pub;
    ros::Publisher joint_03_pub;
    ros::Publisher joint_04_pub;
    ros::Publisher joint_05_pub;
    ros::Publisher joint_06_pub;
    
public:
    IRB120_Commander(ros::NodeHandle n);
    VectorXd ForwardKinematics();
    VectorXd InverseKinematics();
    MatrixXd PathGen();
    MatrixXd Path2Profile_Simultaneous();
    MatrixXd Path2Profile_Separate();
};