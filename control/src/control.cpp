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

#include <geometry_msgs/Pose.h>
#include <string>

int main(int argc, char* argv[])
{
    // Initialize the node & node handle

    ros::init(argc, argv, "arm_control");
    ros::NodeHandle n;

    // Declare local variables

    ros::Rate loop_rate(100);
    int frequency;

    // Read parameters from parameter server

    n.getParam("frequency", frequency);

    // Define publishers, subscribers, services and clients


    ros::AsyncSpinner spinner(1);
    spinner.start();
    

    // Planning Groups
    static const std::string PLANNING_GROUP = "arm";

    // Set up the Move Group Interface using the planning group
    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

    // Add collision objects in virtual world scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // // Raw pointers used to refer to the planning group
    // const moveit::core::JointModelGroup* arm_model_group = arm_move_group_interface.getCurrentState()->getJointModelGroup(arm_planning_group);
    // const moveit::core::JointModelGroup* pincer_model_group = pincer_move_group_interface.getCurrentState()->getJointModelGroup(pincer_planning_group);

    // // Visualization (using MoveItVisualTools)
    // namespace rvt = rviz_visual_tools;
    // moveit_visual_tools::MoveItVisualTools visual_tools("joint1");
    // visual_tools.deleteAllMarkers();

    // visual_tools.trigger();




    // /*************************
    //  * Getting Basic Information
    //  * **********************/

    // ROS_INFO_STREAM("Getting basic information");

    // // We can print the name of the reference frame for this robot.
    // ROS_INFO_NAMED("tutorial", "Arm planning frame: %s", arm_move_group_interface.getPlanningFrame().c_str());
    
    // // We can also print the name of the end-effector link for this group.
    // ROS_INFO_NAMED("tutorial", "Arm end effector link: %s", arm_move_group_interface.getEndEffectorLink().c_str());
    
    // // We can get a list of all the groups in the robot:
    // ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
    // std::copy(arm_move_group_interface.getJointModelGroupNames().begin(),
    //           arm_move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

    // ROS_INFO_STREAM("Finished getting basic information");

    // /*************************
    //  * Start Motion Planning
    //  * **********************/

    // // Plan a motion for this group to a desired pose for the end-effector
    // geometry_msgs::Pose arm_pose;
    // arm_pose.orientation.w = 1.0;
    // arm_pose.position.x = 1.0;
    // arm_pose.position.y = 1.0;
    // arm_pose.position.z = 1.0;
    // pincer_move_group_interface.setPoseTarget(arm_pose);

    // // Call the planner to compute the plan and visualize it
    // moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
    // bool success = (arm_move_group_interface.plan(arm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    // while (ros::ok())
    // {
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }
    ros::shutdown();
    return 0;
}