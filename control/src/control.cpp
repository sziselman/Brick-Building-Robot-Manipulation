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
    

    /**************************
     * Setup
     * ***********************/
    ROS_INFO_STREAM("+++++++ BEGINNING SETUP +++++++");

    // Planning Groups
    static const std::string ARM_PLANNING_GROUP = "arm";
    static const std::string PINCER_PLANNING_GROUP = "pincer";

    // Set up the Move Group Interface using the planning group
    moveit::planning_interface::MoveGroupInterface arm_move_group_interface(ARM_PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface pincer_move_group_interface(PINCER_PLANNING_GROUP);

    // Add collision objects in virtual world scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Raw pointers used to refer to the planning group
    const moveit::core::JointModelGroup* arm_model_group = arm_move_group_interface.getCurrentState()->getJointModelGroup(ARM_PLANNING_GROUP);
    const moveit::core::JointModelGroup* pincer_model_group = pincer_move_group_interface.getCurrentState()->getJointModelGroup(PINCER_PLANNING_GROUP);

    ROS_INFO_STREAM("+++++++ SETUP COMPLETE +++++++");

    /***************************
     * Visualization
     * ************************/

    ROS_INFO_STREAM("+++++++ BEGINNING VISUALIZATION +++++++");

    // provides capabilities for visualizing objects, robots and trajectories in RViz
    namespace rvt = rviz_visual_tools;

    // ARE THESE THE CORRECT JOINT NAMES WHEN VISUALIZING THE ARM AND PINCER??
    moveit_visual_tools::MoveItVisualTools visual_tools("joint1");
    moveit_visual_tools::MoveItVisualTools visual_tools("pincer_joint");

    visual_tools.deleteAllMarkers();

    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.0;
    visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

    visual_tools.trigger();

    ROS_INFO_STREAM("+++++++ VISUALIZATION COMPLETE +++++++");


    // /*************************
    //  * Getting Basic Information
    //  * **********************/

    ROS_INFO_STREAM("+++++++ Getting basic information +++++++");

    // We can print the name of the reference frame for this robot.
    ROS_INFO_NAMED("tutorial", "Arm planning frame: %s", arm_move_group_interface.getPlanningFrame().c_str());
    
    // We can also print the name of the end-effector link for this group.
    ROS_INFO_NAMED("tutorial", "Arm end effector link: %s", arm_move_group_interface.getEndEffectorLink().c_str());
    
    // We can get a list of all the groups in the robot:
    ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
    std::copy(arm_move_group_interface.getJointModelGroupNames().begin(),
              arm_move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

    ROS_INFO_STREAM("+++++++ Finished getting basic information +++++++");

    /*************************
     * Start Motion Planning
     * **********************/

    ROS_INFO_STREAM("+++++++ Starting Motion Planning +++++++");

    // Plan a motion for this group to a desired pose for the end-effector
    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.w = 1.0;
    target_pose1.position.x = 0.25;
    target_pose1.position.y = 0.25;
    target_pose1.position.z = 0.25;
    arm_move_group_interface.setPoseTarget(target_pose1);

    // Call the planner to compute the plan and visualize it
    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    bool success = (arm_move_group_interface.plan(plan1) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    ROS_INFO_STREAM("+++++++ Finishing Motion Planning +++++++");

    /***********************
     * Visualizing Plans
     * ********************/

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
    visual_tools.publishAxisLabeled(target_pose1, "pose1");
    visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(plan1.trajectory_, arm_model_group);

    visual_tools.trigger();

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    // /***********************
    //  * Moving to a pose goal
    //  * ********************/

    // arm_move_group_interface.move();

    /***********************
     * Adding a Collision Object
     * ********************/

    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = arm_move_group_interface.getPlanningFrame();

    collision_object.id = "brick1";

    // Define a box to add to the world
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.092075;
    primitive.dimensions[primitive.BOX_Y] = 0.193675;
    primitive.dimensions[primitive.BOX_Z] = 0.05715;

    // Define a pose for the box (specified relative to frame_id)
    geometry_msgs::Pose brick_pose;
    brick_pose.orientation.w = 1.0;
    brick_pose.position.x = 0.5;
    brick_pose.position.y = 0.5;
    brick_pose.position.z = 0.05715/2;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(brick_pose);
    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);

    planning_scene_interface.addCollisionObjects(collision_objects);

    visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object appears in RViz");

    success = (arm_move_group_interface.plan(plan1) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 6 (pose goal move around cuboid) %s", success ? "" : "FAILED");
    visual_tools.publishText(text_pose, "Obstacle Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(plan1.trajectory_, arm_model_group);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the plan is complete");

    ros::shutdown();
    return 0;
}