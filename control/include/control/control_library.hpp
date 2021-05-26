#ifndef CONTROL_INCLUDE_GUARD_HPP
#define CONTROL_INCLUDE_GUARD_HPP

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float64.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace control_library
{
    constexpr double PI=3.14159265358979323846;

    /// \brief a function that finds a brick designated by location and orientation
    /// \param move_group
    /// \param pose : a geometry_msgs::Pose message that represents the pose of the adroit end-effector
    void moveArm(moveit::planning_interface::MoveGroupInterface& move_group, geometry_msgs::Pose& pose);void movePincer(moveit::planning_interface::MoveGroupInterface& move_group, const moveit::core::JointModelGroup* joint_model_group, double joint_angle);

    /// \brief a function that opens the pincer to a certain angle
    /// \param move_group
    /// \param joint_model_group
    /// \param joint_angle
    void movePincer(moveit::planning_interface::MoveGroupInterface& move_group, const moveit::core::JointModelGroup* joint_model_group, double joint_angle);

    /// \brief a function that moves the arm in stow position while the jackal drives
    /// \param move_group
    /// \param model_group
    /// \param stow_positions
    void stowPosition(moveit::planning_interface::MoveGroupInterface& move_group_interface, const moveit::core::JointModelGroup* model_group, std::vector<double> stow_positions);

    /// \brief a function that gets the pre-grasp pose of the adroit arm
    /// \param transform : the transform recieved from Nathaniel / velodyne
    geometry_msgs::Pose getPreGraspPose(geometry_msgs::TransformStamped& transform);

    /// \brief a function that gets the grasp pose of the adroit arm
    /// \param transform : the transform received from Nathaniel / velodyne
    geometry_msgs::Pose getGraspPose(geometry_msgs::TransformStamped& transform);
}

#endif