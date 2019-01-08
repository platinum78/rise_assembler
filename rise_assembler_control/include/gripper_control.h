#ifndef __GRIPPER_CONTROL_H__
#define __GRIPPER_CONTROL_H__

#include <ros/ros.h>

#define GRIPPER_GRIP 0
#define GRIPPER_OPEN 1

class Gripper_Controller
{
  private:
    int gripperStatus;
    ros::NodeHandle n;

  public:
    Gripper_Controller(ros::NodeHandle n);
    void open();
    void grip();
    void setForce(double force);
    void setPose(double pose);
};

#endif