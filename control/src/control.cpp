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
void addBrick(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface, std::vector<double> brick_loc, std::vector<double> brick_orient, std::string brick_id);
void moveArm(moveit::planning_interface::MoveGroupInterface& move_group_interface, std::vector<double> brick_loc, std::vector<double> brick_orient);
void openPincer(trajectory_msgs::JointTrajectory& posture);
void closePincer(trajectory_msgs::JointTrajectory& posture);
void pickBrick(moveit::planning_interface::MoveGroupInterface& move_group, std::vector<double> brick_loc, std::vector<double> brick_orient, std::string brick_id);
void placeBrick(moveit::planning_interface::MoveGroupInterface& move_group, std::vector<double> brick_loc, std::vector<double>brick_orient, std::string brick_id);


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

    // add collision object / brick
    std::string brick1_id = "brick1";
    addBrick(planning_scene_interface, brick_start_loc, brick_start_orient, brick1_id);
    pincer_visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to move the arm to the brick");

    // find the brick
    // moveArm(arm_move_group_interface, brick_start_loc, brick_start_orient);
    // pincer_visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to pick up the brick");

    // open the pincer to grab the brick
    pickBrick(arm_move_group_interface, brick_start_loc, brick_start_orient, brick1_id);
    pincer_visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to move the arm");
    
    // attach the brick to the robot
    // arm_move_group_interface.attachObject("brick1");
    
    // // move the arm to the brick's goal location
    // moveArm(arm_move_group_interface, brick_goal_loc, brick_goal_orient);
    // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to place the brick down");

    // detach the brick from the robot
    placeBrick(arm_move_group_interface, brick_goal_loc, brick_goal_orient, brick1_id);
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to shut down");
    // arm_move_group_interface.detachObject("brick1");

    ros::shutdown();
    return 0;
}

/// \brief a function that adds a brick as a collision object to the MoveIt planning scene
/// \param planning_scene_interface
/// \param brick_loc : the location of the brick to be added
/// \param brick_orient : the orientation of the brick to be added
void addBrick(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface, std::vector<double> brick_loc, std::vector<double> brick_orient, std::string brick_id)
{
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

/// \brief a function that opens the adroit pincers
void openPincer(trajectory_msgs::JointTrajectory& posture)
{
    // Add pincer finger joints of adroit arm to posture
    posture.joint_names.resize(2);
    posture.joint_names[0] = "pincerfinger_left";
    posture.joint_names[1] = "pincerfinger_right";

    // Set pincer finger joints as open, wide enough for the object to fit
    posture.points.resize(1);
    posture.points[0].positions.resize(2);
    posture.points[0].positions[0] = 0.4;
    posture.points[0].positions[1] = 0.4;
    posture.points[0].time_from_start = ros::Duration(0.5);
}

/// \brief a function that closes the adroit pincers
void closePincer(trajectory_msgs::JointTrajectory& posture)
{
    // add pincer finger joints of adroit arm to posture
    posture.joint_names.resize(2);
    posture.joint_names[0] = "pincerfinger_left";
    posture.joint_names[1] = "pincerfinger_right";

    // set pincer figure joints as closed
    posture.points.resize(1);
    posture.points[0].positions.resize(2);
    posture.points[0].positions[0] = 0.0;
    posture.points[0].positions[1] = 0.0;
    posture.points[0].time_from_start = ros::Duration(0.5);
}

void pickBrick(moveit::planning_interface::MoveGroupInterface& move_group, std::vector<double> brick_loc, std::vector<double> brick_orient, std::string brick_id)
{
    // create a vector of grasps to be attempted
    std::vector<moveit_msgs::Grasp> grasps;
    grasps.resize(1);

    // set the grasp pose
    grasps[0].grasp_pose.header.frame_id = "base_link";
    tf2::Quaternion orientation;
    orientation.setRPY(PI/2, PI/2, brick_orient[2]);

    grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
    grasps[0].grasp_pose.pose.position.x = brick_loc[0];
    grasps[0].grasp_pose.pose.position.y = brick_loc[1];
    grasps[0].grasp_pose.pose.position.z = brick_loc[2];

    // setting pre-grasp approach
    grasps[0].pre_grasp_approach.direction.header.frame_id = "base_link";
    grasps[0].pre_grasp_approach.direction.vector.z = -1.0;
    grasps[0].pre_grasp_approach.min_distance = 0.1;
    grasps[0].pre_grasp_approach.desired_distance = 0.5;

    // setting post-grasp retreat
    grasps[0].post_grasp_retreat.direction.header.frame_id = "base_link";
    grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
    grasps[0].post_grasp_retreat.min_distance = 0.1;
    grasps[0].post_grasp_retreat.desired_distance = 0.5;

    // setting posture of end-effector before grasp
    openPincer(grasps[0].pre_grasp_posture);

    // setting posture of end-effector during grasp
    closePincer(grasps[0].grasp_posture);

    move_group.pick(brick_id, grasps);
}

void placeBrick(moveit::planning_interface::MoveGroupInterface& move_group, std::vector<double> brick_loc, std::vector<double>brick_orient, std::string brick_id)
{
    // a vector of placings to be attempted
    std::vector<moveit_msgs::PlaceLocation> place_location;
    place_location.resize(1);

    // setting place location pose
    place_location[0].place_pose.header.frame_id = "base_link";
    tf2::Quaternion orientation;
    orientation.setRPY(PI/2, PI/2, brick_orient[2]);

    place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

    // placing brick at the exact location of the center of the object
    place_location[0].place_pose.pose.position.x = brick_loc[0];
    place_location[0].place_pose.pose.position.y = brick_loc[1];
    place_location[0].place_pose.pose.position.z = brick_loc[2];

    // setting pre-place approach
    place_location[0].pre_place_approach.direction.header.frame_id = "base_link";
    place_location[0].pre_place_approach.direction.vector.z = -1.0;
    place_location[0].pre_place_approach.min_distance = 0.1;
    place_location[0].pre_place_approach.desired_distance = 0.5;

    // setting post-grasp retreat
    place_location[0].post_place_retreat.direction.header.frame_id = "base_link";
    place_location[0].post_place_retreat.direction.vector.z = 1.0;
    place_location[0].post_place_retreat.min_distance = 0.1;
    place_location[0].post_place_retreat.desired_distance = 0.5;

    // setting posture of end-effector after placing object
    openPincer(place_location[0].post_place_posture);

    move_group.place(brick_id, place_location);
}
