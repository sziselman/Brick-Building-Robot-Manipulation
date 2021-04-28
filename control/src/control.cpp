/// \file control.cpp
/// \brief contains a node called control that will implement motion planning for pick and place functionalities
///
/// PARAMETERS:
/// PUBLISHES:
/// SUBSCRIBES:
/// SERVICES:

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>


int main(int argc, char* argv[])
{
    // Initialize the node & node handle

    ros::init(argc, argv, "control");
    ros::NodeHandle n;

    // Declare local variables

    ros::Rate loop_rate(100);
    int frequency;

    // Read parameters from parameter server

    n.getParam("frequency", frequency);

    // Define publishers, subscribers, services and clients

    // Planning Groups
    static const std::string arm_planning_group = "arm";
    static const std::string pincer_planning_group = "pincer";

    // Set up the Move Group Interface using the planning group
    moveit::planning_interface::MoveGroupInterface arm_move_group_interface(arm_planning_group);
    moveit::planning_interface::MoveGroupInterface pincer_move_group_interface(pincer_planning_group);

    // Add collision objects in virtual world scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Raw pointers used to refer to the planning group
    const moveit::core::JointModelGroup* arm_model_group = arm_move_group_interface.getCurrentState()->getJointModelGroup(arm_planning_group);
    const moveit::core::JointModelGroup* pincer_model_group = pincer_move_group_interface.getCurrentState()->getJointModelGroup(pincer_planning_group);

    // Visualization (using MoveItVisualTools)
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("")
    

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}