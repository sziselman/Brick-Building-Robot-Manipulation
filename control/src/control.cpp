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

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


/********************
 * Global Variables
 * *****************/
std::vector<double> brick_dimensions;
static constexpr double PI=3.14159265358979323846;

/********************
 * Helper Functions
 * *****************/
void addBrick(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface, std::vector<double> brick_loc, std::vector<double> brick_orient);
void moveArm(moveit::planning_interface::MoveGroupInterface& move_group_interface, std::vector<double> brick_loc, std::vector<double> brick_orient);
void openPincer(moveit::planning_interface::MoveGroupInterface& move_group_interface, const moveit::core::JointModelGroup* model_group);

int main(int argc, char* argv[])
{
    // Initialize the node & node handle

    ros::init(argc, argv, "arm_control");
    ros::NodeHandle n;

    // Declare local variables

    ros::Rate loop_rate(100);
    int frequency;
    std::vector<double> brick_start_loc, brick_start_orient;
    std::vector<double> brick_goal_loc, brick_goal_orient;

    // Read parameters from parameter server

    n.getParam("frequency", frequency);
    n.getParam("brick_dimensions", brick_dimensions);
    n.getParam("brick_start_location", brick_start_loc);
    n.getParam("brick_start_orientation", brick_start_orient);
    n.getParam("brick_goal_location", brick_goal_loc);
    n.getParam("brick_goal_orientation", brick_goal_orient);

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
    moveit_visual_tools::MoveItVisualTools pincer_visual_tools("pincer_joint");

    visual_tools.deleteAllMarkers();
    pincer_visual_tools.deleteAllMarkers();

    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.0;
    visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);
    pincer_visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

    visual_tools.trigger();
    pincer_visual_tools.trigger();

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

    // add collision object / brick

    addBrick(planning_scene_interface, brick_start_loc, brick_start_orient);
    pincer_visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to move the arm to the brick");

    // find the brick

    moveArm(arm_move_group_interface, brick_start_loc, brick_start_orient);
    pincer_visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to move the brick to the goal location");

    // // open the pincer to grab the brick
    // openPincer(pincer_move_group_interface, pincer_model_group);

    moveArm(arm_move_group_interface, brick_goal_loc, brick_goal_orient);
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to shut down");

    ros::shutdown();
    return 0;
}

/// \brief a function that adds a brick as a collision object to the MoveIt planning scene
/// \param planning_scene_interface
/// \param brick_loc : the location of the brick to be added
/// \param brick_orient : the orientation of the brick to be added
void addBrick(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface, std::vector<double> brick_loc, std::vector<double> brick_orient)
{
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = "base_link";

    collision_object.id = "brick1";

    // Define a box to add to the world
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = brick_dimensions[0];
    primitive.dimensions[primitive.BOX_Y] = brick_dimensions[1];
    primitive.dimensions[primitive.BOX_Z] = brick_dimensions[2];

    // Define a pose for the box (specified relative to frame_id)
    geometry_msgs::Pose brick_pose;

    tf2::Quaternion brick_quat;
    brick_quat.setRPY(brick_orient[0], brick_orient[1], brick_orient[2]);
    geometry_msgs::Quaternion brick_quat_msg = tf2::toMsg(brick_quat);

    brick_pose.orientation = brick_quat_msg;
    brick_pose.position.x = brick_loc[0];
    brick_pose.position.y = brick_loc[1];
    brick_pose.position.z = brick_loc[2];

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(brick_pose);
    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);

    // add the collision object into the world
    planning_scene_interface.addCollisionObjects(collision_objects);
}

/// \brief a function that finds a brick designated by location and orientation
/// \param move_group_interface
/// \param brick_loc : the location of the brick to find
/// \param brick_orient : the orientation of the btrick to find
void moveArm(moveit::planning_interface::MoveGroupInterface& move_group_interface, std::vector<double> brick_loc, std::vector<double> brick_orient)
{
    ROS_INFO_STREAM("+++++++ Planning to pose goal +++++++");

    // Plan a motion for this group to a desired pose for the end-effector
    geometry_msgs::Pose target_pose;

    tf2::Quaternion pose_quat;
    pose_quat.setRPY(PI/2, PI/2, 0);
    geometry_msgs::Quaternion pose_quat_msg = tf2::toMsg(pose_quat);

    target_pose.orientation = pose_quat_msg;
    target_pose.position.x = brick_loc[0];
    target_pose.position.y = brick_loc[1];
    target_pose.position.z = brick_loc[2] + 0.05;
    move_group_interface.setPoseTarget(target_pose);

    // Call the planner to compute the plan and visualize it
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_interface.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    // /***********************
    //  * Visualizing Plans
    //  * ********************/

    // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
    // visual_tools.publishAxisLabeled(target_pose1, "pose1");
    // visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
    // visual_tools.publishTrajectoryLine(plan1.trajectory_, arm_model_group);

    // visual_tools.trigger();

    // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    // moving to pose goal
    move_group_interface.move();

    ROS_INFO_STREAM("+++++++ Moved the arm!! +++++++");
}

/// \brief a function that opens the adroit pincers
/// \param move_group_interface
/// \param model_group
void openPincer(moveit::planning_interface::MoveGroupInterface& move_group_interface, const moveit::core::JointModelGroup* model_group)
{
    ROS_INFO_STREAM("+++++++ Opening pincers!! +++++++");

    // RobotState object contains current position/velocity/acceleration data
    moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();

    // get current set of joint values for pincer
    std::vector<double> joint_positions;
    current_state->copyJointGroupPositions(model_group, joint_positions);

    for (auto dub : joint_positions)
    {
        ROS_INFO_STREAM(dub);
    }

    // modify joints, plan to the new joint space goal and visualize the plan
    joint_positions[0] = 0.2;
    move_group_interface.setJointValueTarget(joint_positions);

    moveit::planning_interface::MoveGroupInterface::Plan plan2;
    bool success = (move_group_interface.plan(plan2) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan (joint space goal) %s", success ? "" : "FAILED");

    // pincer_visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
    // pincer_visual_tools.publishTrajectoryLine(plan2.trajectory_, pincer_model_group);
    // pincer_visual_tools.trigger();
    // pincer_visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
    
    move_group_interface.move();
}

