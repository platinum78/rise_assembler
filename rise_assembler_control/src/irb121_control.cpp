#include "../include/irb120_control.h"
#include <iostream>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

// Main function
int main(int argc, char **argv)
{
    /*
    ros::init(argc, argv, "irb120_control_node");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    IRB120_Controller irb120(n);
    
    Eigen::MatrixXd H(4, 4);
    H <<    1,     0,       0,      0.3,
            0,     -1,      0,      0,
            0,     0,      -1,      0.3,
            0,          0,      0,      1;
    std::cout << H << std::endl;
    irb120.inverseKinematicsSolver(H);
    irb120.printTheta();
    irb120.copySinglePoint();
    */
    ros::init(argc, argv, "irb120_control_node");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "panda_arm";

    // The :move_group_interface:`MoveGroup` class can be easily
    // setup using just the name of the planning group you would like to control and plan for.
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    // We will use the :planning_scene_interface:`PlanningSceneInterface`
    // class to add and remove collision objects in our "virtual world" scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Raw pointers are frequently used to refer to the planning group for improved performance.
    const robot_state::JointModelGroup *joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // Getting Basic Information
    // ^^^^^^^^^^^^^^^^^^^^^^^^^
    //
    // We can print the name of the reference frame for this robot.
    ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());

    // We can also print the name of the end-effector link for this group.
    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());
}