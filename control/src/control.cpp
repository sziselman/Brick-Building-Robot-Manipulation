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

#include <tf/transform_listener.h>


/********************
 * Global Variables
 * *****************/
std::vector<double> brick_dimensions;
std::vector<double> jackal_dimensions;
std::vector<double> adroit_stow_positions;

static constexpr double PI=3.14159265358979323846;
ros::Publisher pincer_pub;

/********************
 * Helper Functions
 * *****************/
void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface, geometry_msgs::Pose& brick_pose);
void moveArm(moveit::planning_interface::MoveGroupInterface& move_group_interface, geometry_msgs::Pose& pose);
void movePincer(moveit::planning_interface::MoveGroupInterface& move_group, const moveit::core::JointModelGroup* joint_model_group, double joint_angle);
void pincerAngle(std_msgs::Float64 angle);
void stowPosition(moveit::planning_interface::MoveGroupInterface& move_group, const moveit::core::JointModelGroup* model_group);
geometry_msgs::Pose getPreGraspPose(tf::StampedTransform& transform);
geometry_msgs::Pose getGraspPose(tf::StampedTransform& transform);

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

    std_msgs::Float64 open_angle, grasp_angle;
    open_angle.data = 0.90;
    grasp_angle.data = 0.60;
    
    // Read parameters from parameter server

    n.getParam("frequency", frequency);
    n.getParam("brick_dimensions", brick_dimensions);
    n.getParam("jackal_dimensions", jackal_dimensions);
    n.getParam("brick_start_location", brick_start_loc);
    n.getParam("brick_start_orientation", brick_start_orient);
    n.getParam("brick_goal_location", brick_goal_loc);
    n.getParam("brick_goal_orientation", brick_goal_orient);
    n.getParam("adroit_stow_positions", adroit_stow_positions);

    // Publishers, subscribers, listeners, services, etc.

    pincer_pub = n.advertise<std_msgs::Float64>("/hdt_arm/pincer_joint_position_controller/command", 10);
    tf::TransformListener brick_listener;

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

    visual_tools.deleteAllMarkers();

    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.0;
    visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

    visual_tools.trigger();

    ROS_INFO_STREAM("+++++++ VISUALIZATION COMPLETE +++++++");

    /***************************
     * Pick + Place
     * ************************/

    ROS_INFO_STREAM("+++++++ BEGINNING PICK + PLACE +++++++");

    // listen for a transform broadcasted by Nathaniel + Velodyne
    // create pose for collision object
    tf::StampedTransform brick_transform;
    brick_listener.lookupTransform("/hdt_arm", "/brick", ros::Time(0), brick_transform);

    // geometry_msgs::Pose brick_pose;
    // geometry_msgs::Quaternion brick_quat_msg;
    // quaternionTFToMsg(brick_transform.getRotation(), brick_quat_msg);

    // brick_pose.orientation = brick_quat_msg;
    // brick_pose.position.x = brick_transform.getOrigin().x();
    // brick_pose.position.y = brick_transform.getOrigin().y();
    // brick_pose.position.z = brick_transform.getOrigin().z();

    // ADD COLLISION OBJECT / BRICK
    geometry_msgs::Pose brick_start_pose;
    tf2::Quaternion brick_quat;
    brick_quat.setRPY(brick_start_orient[0], brick_start_orient[1], brick_start_orient[2]);
    brick_start_pose.orientation = tf2::toMsg(brick_quat);
    brick_start_pose.position.x = brick_start_loc[0];
    brick_start_pose.position.y = brick_start_loc[1];
    brick_start_pose.position.z = brick_start_loc[2];

    addCollisionObjects(planning_scene_interface, brick_start_pose);
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to move the arm to pre-grasp pose");

    // MOVE ARM TO THE BRICK START LOCATION
    geometry_msgs::Pose pre_grasp_pose;
    tf2::Quaternion pre_grasp_quat;
    pre_grasp_quat.setRPY(PI/2, PI/2, brick_start_orient[2]);
    pre_grasp_pose.orientation = tf2::toMsg(pre_grasp_quat);
    pre_grasp_pose.position.x = brick_start_loc[0];
    pre_grasp_pose.position.y = brick_start_loc[1];
    pre_grasp_pose.position.z = brick_start_loc[2] + 0.1;

    moveArm(arm_move_group_interface, pre_grasp_pose);
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to open the pincers");

    // OPEN THE PINCERS
    // // movePincer(pincer_move_group_interface, pincer_model_group, 0.9);
    pincerAngle(open_angle);
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to move the arm to grasp pose");

    geometry_msgs::Pose grasp_pose;
    tf2::Quaternion grasp_quat;
    grasp_quat.setRPY(PI/2, PI/2, brick_start_orient[2]);
    grasp_pose.orientation = tf2::toMsg(grasp_quat);
    grasp_pose.position.x = brick_start_loc[0];
    grasp_pose.position.y = brick_start_loc[1];
    grasp_pose.position.z = brick_start_loc[2] - 0.01;

    moveArm(arm_move_group_interface, grasp_pose);
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to close the pincers");

    // CLOSE THE PINCERS ON THE BRICK
    // movePincer(pincer_move_group_interface, pincer_model_group, 0.3);
    pincerAngle(grasp_angle);
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to move attach the brick");
    arm_move_group_interface.attachObject("brick");
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to move the adroit arm in stow position");

    // PLACE THE ARM IN STOW POSITION
    stowPosition(arm_move_group_interface, arm_model_group);
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to move the brick to the goal location");

    // MOVE ARM TO BRICK GOAL LOCATION
    geometry_msgs::Pose place_pose;
    tf2::Quaternion place_quat;
    place_quat.setRPY(PI/2, PI/2, brick_goal_orient[2]);
    place_pose.orientation = tf2::toMsg(grasp_quat);
    place_pose.position.x = brick_goal_loc[0];
    place_pose.position.y = brick_goal_loc[1];
    place_pose.position.z = brick_goal_loc[2] - 0.01;
    moveArm(arm_move_group_interface, place_pose);
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to open the pincers");

    pincerAngle(open_angle);
    // movePincer(pincer_move_group_interface, pincer_model_group, 0.9);
    arm_move_group_interface.detachObject("brick");
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to end the pick and place");

    ROS_INFO_STREAM("+++++++ COMPLETED PICK + PLACE +++++++");

    ros::shutdown();
    return 0;
}

/// \brief a function that adds a brick as a collision object to the MoveIt planning scene
/// \param planning_scene_interface
/// \param brick_pose : a geometry_msgs::Pose message that represents the brick's pose
/// \param brick_orient : the orientation of the brick to be added
void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface, geometry_msgs::Pose& brick_pose)
{
    // Create vector to hold bricks + jackal
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(2);

    /**************************
     * Add the Jackal as a collision object
     * ***********************/
    collision_objects[0].id = "jackal";
    collision_objects[0].header.frame_id = "base_link";

    // Define the jackal primitive and its dimensions
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    for (int i = 0; i < jackal_dimensions.size(); i++)
    {
        collision_objects[0].primitives[0].dimensions[i] = jackal_dimensions[i];
    }

    // Define a pose for the jackal
    collision_objects[0].primitive_poses.resize(1);
    tf2::Quaternion jackal_quat;
    jackal_quat.setRPY(0.0, 0.0, 0.0);

    collision_objects[0].primitive_poses[0].orientation = tf2::toMsg(jackal_quat);
    collision_objects[0].primitive_poses[0].position.x = 0.356 - jackal_dimensions[0]/2;
    collision_objects[0].primitive_poses[0].position.y = 0.0;
    collision_objects[0].primitive_poses[0].position.z = -jackal_dimensions[2]/2;

    collision_objects[0].operation = collision_objects[0].ADD;

    /**************************
     * Add the brick as a collision object
     * ***********************/
    collision_objects[1].id = "brick";
    collision_objects[1].header.frame_id = "base_link";

    // Define the brick primitive its dimensions
    collision_objects[1].primitives.resize(1);
    collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
    collision_objects[1].primitives[0].dimensions.resize(3);
    for (int i = 0; i < brick_dimensions.size(); i++)
    {
        collision_objects[1].primitives[0].dimensions[i] = brick_dimensions[i];
    }

    // Define a pose for the brick (inputted in the function)
    collision_objects[1].primitive_poses.resize(1);
    collision_objects[1].primitive_poses[0] = brick_pose;


    collision_objects[1].operation = collision_objects[1].ADD;

    // add the collision object into the world
    planning_scene_interface.applyCollisionObjects(collision_objects);
}

/// \brief a function that finds a brick designated by location and orientation
/// \param move_group_interface
/// \param pose : a geometry_msgs::Pose message that represents the pose of the adroit end-effector
void moveArm(moveit::planning_interface::MoveGroupInterface& move_group_interface, geometry_msgs::Pose& pose)
{
    ROS_INFO_STREAM("+++++++ Planning to pose goal +++++++");

    move_group_interface.setPoseTarget(pose);

    // Call the planner to compute the plan and visualize it
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_interface.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan (pose goal) %s", success ? "" : "FAILED");

    // moving to pose goal
    move_group_interface.move();
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

/// \brief a function that publishes a position for the pincer_joint
/// \param angle : a std_msgs::Float64 that represents the position
void pincerAngle(std_msgs::Float64 angle)
{
    pincer_pub.publish(angle);
}

/// \brief a function that places the adroit arm in a "stow away" position while the jackal is moving
/// \param move_group : a moveit::planning_interface::MoveGroupInterface that is used to plan and execute the joint-space goal
void stowPosition(moveit::planning_interface::MoveGroupInterface& move_group_interface, const moveit::core::JointModelGroup* model_group)
{
    // create a pointer that references the robot's state
    moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(model_group, joint_group_positions);

    // modify the joints, plan to the new joint-space goal and visualize
    for (int i = 0; i < joint_group_positions.size(); i++)
    {
        joint_group_positions[i] = adroit_stow_positions[i];
    }
    move_group_interface.setJointValueTarget(joint_group_positions);

    // Call the planner to compute the plan
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_interface.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan (joint-space goal) %s", success ? "" : "FAILED");

    move_group_interface.move();
}

geometry_msgs::Pose getPreGraspPose(tf::StampedTransform& transform)
{
    // get the rotation (quaternion) from the transform
    tf::Quaternion quat = transform.getRotation();
    // get the yaw of the quaternion
    double yaw = tf::getYaw(quat);
    // create pre-grasp pose
    tf2::Quaternion pre_grasp_quat;
    pre_grasp_quat.setRPY(PI/2, PI/2, yaw);

    geometry_msgs::Pose pose;
    pose.orientation = tf2::toMsg(pre_grasp_quat);
    pose.position.x = transform.getOrigin().x();
    pose.position.y = transform.getOrigin().y();
    pose.position.z = transform.getOrigin().z() + 0.05;

    return pose;
}

geometry_msgs::Pose getGraspPose(tf::StampedTransform& transform)
{
    // get the rotation (quaternion) from the transform
    tf::Quaternion quat = transform.getRotation();
    // get the yaw of the quaternion
    double yaw = tf::getYaw(quat);
    // create pre-grasp pose
    tf2::Quaternion pre_grasp_quat;
    pre_grasp_quat.setRPY(PI/2, PI/2, yaw);

    geometry_msgs::Pose pose;
    pose.orientation = tf2::toMsg(pre_grasp_quat);
    pose.position.x = transform.getOrigin().x();
    pose.position.y = transform.getOrigin().y();
    pose.position.z = transform.getOrigin().z() - 0.01;

    return pose;
}
