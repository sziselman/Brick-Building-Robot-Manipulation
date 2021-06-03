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
#include <geometry_msgs/TransformStamped.h>

#include <string>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Float64.h>

#include <tf2_ros/transform_listener.h>

#include <control/control_library.hpp>

#include <std_srvs/Empty.h>

/********************
 * Global Variables
 * *****************/
static std::vector<double> brick_dimensions;
static std::vector<double> jackal_dimensions;
static std::vector<double> adroit_stow_positions;

static ros::Publisher pincer_pub;

enum State {Idle, Stow, PreGraspPose, GraspPose, PlacePose};

static State current_state = Idle;

static double upperbound_z, floor_z;

/********************
 * Helper Functions
 * *****************/
void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface, geometry_msgs::Pose& brick_pose);
void pincerAngle(std_msgs::Float64 angle);
bool stow_position(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
bool open_pincers(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
bool close_pincers(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
bool pregrasp_position(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
bool grasp_position(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
bool place_position(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

int main(int argc, char* argv[])
{
    using namespace control_library;
    
    // Initialize the node & node handle

    ros::init(argc, argv, "arm_control");
    ros::NodeHandle n;

    // Declare local variables

    ros::Rate loop_rate(100);
    int frequency;
    std::vector<double> brick_start_loc, brick_start_orient;
    std::vector<double> brick_goal_loc, brick_goal_orient;

    std_msgs::Float64 open_angle, grasp_angle, close_angle;
    open_angle.data = 0.90;
    grasp_angle.data = 0.50;
    close_angle.data = 0.0;
    
    // Read parameters from parameter server

    n.getParam("frequency", frequency);
    n.getParam("brick_dimensions", brick_dimensions);
    n.getParam("jackal_dimensions", jackal_dimensions);
    n.getParam("brick_start_location", brick_start_loc);
    n.getParam("brick_start_orientation", brick_start_orient);
    n.getParam("brick_goal_location", brick_goal_loc);
    n.getParam("brick_goal_orientation", brick_goal_orient);
    n.getParam("adroit_stow_positions", adroit_stow_positions);
    n.getParam("upperbound_plane_z", upperbound_z);
    n.getParam("floor_plane_z", floor_z);

    // Publishers, subscribers, listeners, services, etc.

    pincer_pub = n.advertise<std_msgs::Float64>("/hdt_arm/pincer_joint_position_controller/command", 10);
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener brick_listener(tfBuffer);

    ros::ServiceServer stow_pos_service = n.advertiseService("/stow_position", stow_position);
    ros::ServiceClient stow_pos_client = n.serviceClient<std_srvs::Empty>("/stow_position");
    
    ros::ServiceServer pregrasp_service = n.advertiseService("/pregrasp_position", pregrasp_position);
    ros::ServiceClient pregrasp_client = n.serviceClient<std_srvs::Empty>("/pregrasp_position");

    ros::ServiceServer grasp_service = n.advertiseService("/grasp_position", grasp_position);
    ros::ServiceClient grasp_client = n.serviceClient<std_srvs::Empty>("/grasp_position");

    ros::ServiceServer place_service = n.advertiseService("/place_position", place_position);
    ros::ServiceClient place_client = n.serviceClient<std_srvs::Empty>("/place_position");

    ros::ServiceServer close_pincers_service = n.advertiseService("/close_pincers", close_pincers);
    ros::ServiceClient close_pincers_client = n.serviceClient<std_srvs::Empty>("/close_pincers");

    ros::ServiceServer open_pincers_service = n.advertiseService("/open_pincers", open_pincers);
    ros::ServiceClient open_pincers_client = n.serviceClient<std_srvs::Empty>("/open_pincers");


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

    // // listen for a transform broadcasted by Nathaniel + Velodyne
    // // create pose for collision object

    // geometry_msgs::TransformStamped brick_transform;
    // brick_transform = tfBuffer.lookupTransform("/hdt_arm", "/brick", ros::Time(0));

    // geometry_msgs::Pose brick_pose;

    // brick_pose.orientation = brick_transform.transform.rotation;
    // brick_pose.position.x = brick_transform.transform.translation.x;
    // brick_pose.position.y = brick_transform.transform.translation.y;
    // brick_pose.position.z = brick_transform.transform.translation.z;

    // ADD COLLISION OBJECT / BRICK
    geometry_msgs::Pose brick_start_pose;
    tf2::Quaternion brick_quat;
    brick_quat.setRPY(brick_start_orient[0], brick_start_orient[1], brick_start_orient[2]);
    brick_start_pose.orientation = tf2::toMsg(brick_quat);
    brick_start_pose.position.x = brick_start_loc[0];
    brick_start_pose.position.y = brick_start_loc[1];
    brick_start_pose.position.z = brick_start_loc[2];

    stowPosition(arm_move_group_interface, arm_model_group, adroit_stow_positions);
    addCollisionObjects(planning_scene_interface, brick_start_pose);

    while (ros::ok())
    {
        switch (current_state)
        {
            case Idle:
                {
                    break;
                }

            case Stow:
                {
                    ROS_INFO_STREAM("Moving Adroit stow position...");
                    stowPosition(arm_move_group_interface, arm_model_group, adroit_stow_positions);
                    current_state = Idle;
                    break;
                }

            case PreGraspPose:
                {
                    ROS_INFO_STREAM("Moving Pincer to pre-grasp position...");
                    geometry_msgs::Pose pre_grasp_pose;
                    tf2::Quaternion pre_grasp_quat;
                    pre_grasp_quat.setRPY(PI/2, PI/2, brick_start_orient[2]);
                    pre_grasp_pose.orientation = tf2::toMsg(pre_grasp_quat);
                    pre_grasp_pose.position.x = brick_start_loc[0];
                    pre_grasp_pose.position.y = brick_start_loc[1];
                    pre_grasp_pose.position.z = brick_start_loc[2] + 0.1;
                    moveArm(arm_move_group_interface, pre_grasp_pose);
                    current_state = Idle;
                    break;
                }
            
            case GraspPose:
                {
                    ROS_INFO_STREAM("Moving Pincer to grasp position...");
                    geometry_msgs::Pose grasp_pose;
                    tf2::Quaternion grasp_quat;
                    grasp_quat.setRPY(PI/2, PI/2, brick_start_orient[2]);
                    grasp_pose.orientation = tf2::toMsg(grasp_quat);
                    grasp_pose.position.x = brick_start_loc[0];
                    grasp_pose.position.y = brick_start_loc[1];
                    grasp_pose.position.z = brick_start_loc[2] - 0.01;
                    moveArm(arm_move_group_interface, grasp_pose);
                    current_state = Idle;
                    break;
                }

            case PlacePose:
                {
                    ROS_INFO_STREAM("Placing the brick in goal location...");
                    geometry_msgs::Pose place_pose;
                    tf2::Quaternion place_quat;
                    place_quat.setRPY(PI/2, PI/2, brick_goal_orient[2]);
                    place_pose.orientation = tf2::toMsg(place_quat);
                    place_pose.position.x = brick_goal_loc[0];
                    place_pose.position.y = brick_goal_loc[1];
                    place_pose.position.z = brick_goal_loc[2] - 0.01;
                    moveArm(arm_move_group_interface, place_pose);
                    current_state = Idle;
                    break;
                }
        }
    }
    
    ros::waitForShutdown();
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
    // collision_objects.resize(2);
    collision_objects.resize(3);

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

    /**************************
     * Add the upper bounding plane as a collision object
     * ***********************/
    collision_objects[2].id = "upper_bound_plane";
    collision_objects[2].header.frame_id = "base_link";

    // Define the upper bounding plane's coefficients
    collision_objects[2].planes.resize(1);
    collision_objects[2].planes[0].coef[0] = 0;
    collision_objects[2].planes[0].coef[1] = 0;
    collision_objects[2].planes[0].coef[2] = 1;
    collision_objects[2].planes[0].coef[3] = 0;

    // Define a pose for the upper bounding plane
    collision_objects[2].plane_poses.resize(1);
    tf2::Quaternion plane_quat;
    plane_quat.setRPY(0.0, 0.0, 0.0);

    collision_objects[2].plane_poses[0].orientation = tf2::toMsg(plane_quat);
    collision_objects[2].plane_poses[0].position.x = 0.0;
    collision_objects[2].plane_poses[0].position.y = 0.0;
    collision_objects[2].plane_poses[0].position.z = upperbound_z;
    collision_objects[2].plane_poses[0].orientation.w = 1.0;

    collision_objects[2].operation = collision_objects[2].ADD;

    // add the collision object into the world
    planning_scene_interface.applyCollisionObjects(collision_objects);
}

/// \brief a callback function for service that places the arm in its stow position
/// \param req : empty request
/// \param res : empty response
bool stow_position(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    current_state = Stow;
    return true;
}

/// \brief a callback function for service that places the arm in pre-grasp position above brick
/// \param req : empty request
/// \param res : empty response
bool pregrasp_position(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    current_state = PreGraspPose;
    return true;
}

/// \brief a callback function for service that places the arm in grasp position
/// \param req : empty request
/// \param res : empty response
bool grasp_position(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    current_state = GraspPose;
    return true;
}

/// \brief a callback function for service that places the arm in goal brick location
/// \param req : empty request
/// \param res : empty response
bool place_position(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    current_state = PlacePose;
    return true;
}

/// \brief a callback function for service that opens the pincers
/// \param req : empty request
/// \param res : empty response
bool open_pincers(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    std_msgs::Float64 open_angle;
    open_angle.data = 0.90;
    pincer_pub.publish(open_angle);
    return true;
}

/// \brief a callback function for service that closes the pincers
/// \param req : empty request
/// \param res : empty response
bool close_pincers(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    std_msgs::Float64 close_angle;
    close_angle.data = 0.0;
    pincer_pub.publish(close_angle);
    return true;
}
