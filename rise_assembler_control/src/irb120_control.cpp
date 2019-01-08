#include "../include/irb120_control.h"

// Main function
int main(int argc, char** argv)
{
    ros::init(argc, argv, "irb120_control_node");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    IRB120_Controller irb120(n);
    
    while (!ros::isShuttingDown())
    {
        
        loop_rate.sleep();
        ros::spin();
    }
}