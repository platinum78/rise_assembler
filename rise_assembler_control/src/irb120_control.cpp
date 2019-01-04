#include "../include/irb120_control.h"

// IRB120_Controller class initializer
IRB120_Controller::IRB120_Controller(ros::NodeHandle n)
{
    ////////////////////////////////////////////////////////////////////////////////
    // Initialize parameters and input robot dimensions
    ////////////////////////////////////////////////////////////////////////////////
    
    // Static parameters
    d = VectorXd(6); d << 290, 0, 0, 302, 0, 72;
    a = VectorXd(6); a << 0, 270, 70, 0, 0, 0;
    alpha = VectorXd(6); alpha << 90, 0, 90, -90, 90, 0;
    alpha *= (PI/180);        // Convert from degrees to radians
    
    // Dynamic parameters
    theta = VectorXd(6);
    theta_target = VectorXd(6);
    endEffectorPos = VectorXd(6);
    jointRotationMatrix = MatrixXd(3, 3);
    jointTransMatrix = MatrixXd(4, 4);
    endEffectorTransMatrix = MatrixXd(4, 4);


    ////////////////////////////////////////////////////////////////////////////////
    // Initialize ROS communications
    ////////////////////////////////////////////////////////////////////////////////
    
    // ROS publishers
    joint_01_pub = n.advertise<std_msgs::Float64>("/irb120/joint_position_controller/01/command", 1000);
    joint_02_pub = n.advertise<std_msgs::Float64>("/irb120/joint_position_controller/02/command", 1000);
    joint_03_pub = n.advertise<std_msgs::Float64>("/irb120/joint_position_controller/03/command", 1000);
    joint_04_pub = n.advertise<std_msgs::Float64>("/irb120/joint_position_controller/04/command", 1000);
    joint_05_pub = n.advertise<std_msgs::Float64>("/irb120/joint_position_controller/05/command", 1000);
    joint_06_pub = n.advertise<std_msgs::Float64>("/irb120/joint_position_controller/06/command", 1000);

    // ROS service servers
    // move_robot_service = n.advertiseService("move_robot", moveRobot);
}

// Get homogeneous transformation for single joint
MatrixXd IRB120_Controller::getTransMatrix(double theta, double d_, double a_, double alpha_)
{
    MatrixXd T01(4, 4);
    T01 <<  cos(theta), -sin(theta) * cos(alpha_),  sin(theta) * sin(alpha_),   a_ * cos(theta),
            sin(theta), cos(theta) * cos(alpha_),   -cos(theta) * sin(alpha_),  a_ * sin(theta),
            0,          sin(alpha_),                cos(alpha_),                d_,
            0,          0,                          0,                          1;
    return T01;
}

// Get homogeneous transformation matrix for end effector
MatrixXd IRB120_Controller::getHomoTransMatrix(MatrixXd theta, VectorXd d, VectorXd a, VectorXd alpha)
{
    MatrixXd T01 = getTransMatrix(theta(0, 0), d(0), a(0), alpha(0));
    MatrixXd T12 = getTransMatrix(theta(0, 1), d(1), a(1), alpha(1));
    MatrixXd T23 = getTransMatrix(theta(0, 2), d(2), a(2), alpha(2));
    MatrixXd T34 = getTransMatrix(theta(0, 3), d(3), a(3), alpha(3));
    MatrixXd T45 = getTransMatrix(theta(0, 4), d(4), a(4), alpha(4));
    MatrixXd T56 = getTransMatrix(theta(0, 5), d(5), a(5), alpha(5));
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
void IRB120_Controller::forwardKinematics(VectorXd theta_)
{
    theta = theta_;
    theta(0) = theta(0) * (PI / 180);
    theta(1) = (theta(1) + 90) * (PI / 180);
    theta(2) = theta(2) * (PI / 180);
    theta(3) = theta(3) * (PI / 180);
    theta(4) = theta(4) * (PI / 180);
    theta(5) = theta(5) * (PI / 180);
}

// Inverse kinematics solver
void IRB120_Controller::inverseKinematics(MatrixXd H)
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
    
}

// Convert end-effector path to joint profile:
// start and stop at same time
MatrixXd IRB120_Controller::path2Profile()
{

}

void IRB120_Controller::publishPosition()
{
    // Create ROS messages
    joint_01_msg.data = theta(0);
    joint_02_msg.data = theta(1);
    joint_03_msg.data = theta(2);
    joint_04_msg.data = theta(3);
    joint_05_msg.data = theta(4);
    joint_06_msg.data = theta(5);

    // Publish each message
    joint_01_pub.publish(joint_01_msg);
    joint_02_pub.publish(joint_02_msg);
    joint_03_pub.publish(joint_03_msg);
    joint_04_pub.publish(joint_04_msg);
    joint_05_pub.publish(joint_05_msg);
    joint_06_pub.publish(joint_06_msg);
}

bool moveRobot()
{
    
}


// Main function
int main(int argc, char** argv)
{
    ros::init(argc, argv, "irb120_control_node");
    ros::NodeHandle n;
    
    MatrixXd X, Y, Joint_angles;
    VectorXd theta(6);
    VectorXd d(6);
    VectorXd a(6);
    VectorXd alpha(6);

    IRB120_Controller irb120(n);
    VectorXd y(3);
    VectorXd x(3);
    MatrixXd A1(1001, 6);

    // H matrix for making the robot to go to the BGA Workspace center

    double theta_deg = 0;
    MatrixXd orientation(3, 3);

    orientation(0, 0) = 1;  orientation(0, 1) = 0;  orientation(0, 2) = 0;
    orientation(1, 0) = 0;  orientation(1, 1) = -1; orientation(1, 2) = 0;
    orientation(2, 0) = 0;  orientation(2, 1) = 0;  orientation(2, 2) = -1;

    MatrixXd position(3, 1);
    position(0, 0) = 350;
    position(1, 0) = -200;
    position(2, 0) = 400;
    MatrixXd H(4, 4);

    for (int j = 0; j < 4; j++)
    {
        for (int k = 0; k < 4; k++)
        {
            if (j < 3 && k < 3)
                H(j, k) = orientation(j, k);
            if (j < 3 && k == 3)
                H(j, k) = position(j, 0);
            if (k < 2 && j == 3)
                H(j, k) = 0;
            if (k == 3 && j == 3)
                H(j, k) = 1;
        }
    }

    MatrixXd home(1, 6);
    irb120.inverseKinematics(H);
    cout << home << endl;
    ros::Rate loop_rate(10);
    for (int i = 1; i < 10; i++)
    {
        irb120.moveRobot();
        loop_rate.sleep();
    }

    ros::spin();
    
}