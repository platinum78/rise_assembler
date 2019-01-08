#ifndef __GRIPPER_CONTROL_CPP__
#define __GRIPPER_CONTROL_CPP__

#include "../include/gripper_control.h"

Gripper_Controller::Gripper_Controller(ros::NodeHandle handle)
{
    gripperStatus = GRIPPER_OPEN;
    n = handle;
}

void Gripper_Controller::open()
{

}

void Gripper_Controller::grip()
{

}

void Gripper_Controller::setForce(double force)
{

}

void Gripper_Controller::setPose(double pose)
{

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gripper_control_node");
    ros::NodeHandle n;
    Gripper_Controller gripper(n);
}

#endif
