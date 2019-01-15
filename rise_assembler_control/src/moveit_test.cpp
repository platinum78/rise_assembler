#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <tf/transform_datatypes.h>

#include <iostream>

// #include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "moveit_test_node");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "abb_irb120";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    geometry_msgs::Pose targetPose;
    targetPose.orientation.w = 1;
    targetPose.orientation.x = targetPose.orientation.y = targetPose.orientation.z = 0.0;
    targetPose.position.x = 0.38;
    targetPose.position.y = 0.0;
    targetPose.position.z = 0.5;
    move_group.setPoseTarget(targetPose);

    ROS_INFO("Attempting to find path to target position...");
    moveit::planning_interface::MoveGroupInterface::Plan trajectoryPlan;
    bool success = (move_group.plan(trajectoryPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success) ROS_INFO("Succeeded to find path to target position!");
    else ROS_WARN("Failed to find path to target position.");
    std::cout << trajectoryPlan.planning_time_ << std::endl;


    ros::shutdown();
    return 0;
}