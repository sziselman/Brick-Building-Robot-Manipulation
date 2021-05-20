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
#include <moveit_msgs/Grasp.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <geometry_msgs/Pose.h>
#include <string>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Float64.h>


/********************
 * Global Variables
 * *****************/
std::vector<double> brick_dimensions;
std::vector<double> jackal_dimensions;
static constexpr double PI=3.14159265358979323846;
ros::Publisher pincer_pub;

/********************
 * Helper Functions
 * *****************/
void addBrick(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface, std::vector<double> brick_loc, std::vector<double> brick_orient, std::string brick_id);
void moveArm(moveit::planning_interface::MoveGroupInterface& move_group_interface, std::vector<double> brick_loc, std::vector<double> brick_orient);
void movePincer(moveit::planning_interface::MoveGroupInterface& move_group, const moveit::core::JointModelGroup* joint_model_group, double joint_angle);
void pincerAngle(std_msgs::Float64 angle);

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
    std_msgs::Float64 open_angle;
    open_angle.data = 0.90;

    // Read parameters from parameter server

    n.getParam("frequency", frequency);
    n.getParam("brick_dimensions", brick_dimensions);
    n.getParam("jackal_dimensions", jackal_dimensions);
    n.getParam("brick_start_location", brick_start_loc);
    n.getParam("brick_start_orientation", brick_start_orient);
    n.getParam("brick_goal_location", brick_goal_loc);
    n.getParam("brick_goal_orientation", brick_goal_orient);

    pincer_pub = n.advertise<std_msgs::Float64>("/hdt_arm/pincer_joint_position_controller/command", 10);

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
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
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

    // ADD COLLISION OBJECT / BRICK
    std::string brick1_id = "brick1";
    addBrick(planning_scene_interface, brick_start_loc, brick_start_orient, brick1_id);
    pincer_visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to move the arm to the brick");

    // MOVE ARM TO THE BRICK START LOCATION
    moveArm(arm_move_group_interface, brick_start_loc, brick_start_orient);
    pincer_visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to open the pincers");

    // OPEN THE PINCERS
    // movePincer(pincer_move_group_interface, pincer_model_group, 0.9);
    pincerAngle(open_angle);
    pincer_visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to close the pincers");

    // // CLOSE THE PINCERS ON THE BRICK
    // movePincer(pincer_move_group_interface, pincer_model_group, 0.3);

    // ATTACH THE BRICK TO THE ROBOT
    arm_move_group_interface.attachObject("brick1");
    pincer_visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to move the arm to the goal location");
    
    // MOVE ARM TO BRICK GOAL LOCATION
    moveArm(arm_move_group_interface, brick_goal_loc, brick_goal_orient);
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to place the brick");

    // DETACH THE BRICK FROM THE ROBOT
    arm_move_group_interface.detachObject("brick1");
    // movePincer(pincer_move_group_interface, pincer_model_group, 0.9);
    pincer_visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to end the pick and place");

    ros::shutdown();
    return 0;
}

/// \brief a function that adds a brick as a collision object to the MoveIt planning scene
/// \param planning_scene_interface
/// \param brick_loc : the location of the brick to be added
/// \param brick_orient : the orientation of the brick to be added
void addBrick(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface, std::vector<double> brick_loc, std::vector<double> brick_orient, std::string brick_id)
{
    moveit_msgs::CollisionObject jackal_collision_object;
    jackal_collision_object.header.frame_id = "base_link";
    jackal_collision_object.id = "jackal";

    // Define a box to represent the jackal
    shape_msgs::SolidPrimitive jackal;
    jackal.type = jackal.BOX;
    jackal.dimensions.resize(3);
    jackal.dimensions[jackal.BOX_X] = jackal_dimensions[0];
    jackal.dimensions[jackal.BOX_Y] = jackal_dimensions[1];
    jackal.dimensions[jackal.BOX_Z] = jackal_dimensions[2];

    // Define a pose for the jackal
    geometry_msgs::Pose jackal_pose;

    tf2::Quaternion jackal_quat;
    jackal_quat.setRPY(0.0, 0.0, 0.0);

    jackal_pose.orientation = tf2::toMsg(jackal_quat);
    jackal_pose.position.x = 0.356 - jackal_dimensions[0]/2;
    jackal_pose.position.y = 0.0;
    jackal_pose.position.z = -jackal_dimensions[2]/2;

    jackal_collision_object.primitives.push_back(jackal);
    jackal_collision_object.primitive_poses.push_back(jackal_pose);
    jackal_collision_object.operation = jackal_collision_object.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(jackal_collision_object);

    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = "base_link";

    collision_object.id = brick_id;

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

    brick_pose.orientation = tf2::toMsg(brick_quat);
    brick_pose.position.x = brick_loc[0];
    brick_pose.position.y = brick_loc[1];
    brick_pose.position.z = brick_loc[2];

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(brick_pose);
    collision_object.operation = collision_object.ADD;

    collision_objects.push_back(jackal_collision_object);

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
    pose_quat.setRPY(PI/2, PI/2, brick_orient[2]);
    geometry_msgs::Quaternion pose_quat_msg = tf2::toMsg(pose_quat);

    target_pose.orientation = pose_quat_msg;
    target_pose.position.x = brick_loc[0];
    target_pose.position.y = brick_loc[1];
    target_pose.position.z = brick_loc[2] + 0.1;
    move_group_interface.setPoseTarget(target_pose);

    // Call the planner to compute the plan and visualize it
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_interface.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    // moving to pose goal
    move_group_interface.move();

    ROS_INFO_STREAM("+++++++ Moved the arm!! +++++++");
}

void movePincer(moveit::planning_interface::MoveGroupInterface& move_group, const moveit::core::JointModelGroup* joint_model_group, double joint_angle)
{
    // a pointer that references the current robot's states
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    // get the current set of joint values for the group
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    // modify the first joint position
    joint_group_positions[0] =joint_angle;

    // pass the desired joint positions to move_group as goal for planning
    move_group.setJointValueTarget(joint_group_positions);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    move_group.move(); 
}

void pincerAngle(std_msgs::Float64 angle)
{
    pincer_pub.publish(angle);
}