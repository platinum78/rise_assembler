#include "../include/irb120_control.h"

// Main function
int main(int argc, char** argv)
{
    ros::init(argc, argv, "irb120_control_node");
    ros::NodeHandle n;

    IRB120_Controller irb120(n);
    VectorXd y(3);
    VectorXd x(3);
    // MatrixXd A1(1001, 6);

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