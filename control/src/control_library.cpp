#include "control/control_library.hpp"

namespace control_library
{
    void moveArm(moveit::planning_interface::MoveGroupInterface& move_group, geometry_msgs::Pose& pose)
    {
        // ROS_INFO_STREAM("+++++++ Planning to pose goal +++++++");

        move_group.setPoseTarget(pose);

        // Call the planner to compute the plan and visualize it
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        // bool success = (move_group_interface.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        // ROS_INFO_NAMED("tutorial", "Visualizing plan (pose goal) %s", success ? "" : "FAILED");

        // moving to pose goal
        move_group.move();
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

    void stowPosition(moveit::planning_interface::MoveGroupInterface& move_group_interface, const moveit::core::JointModelGroup* model_group, std::vector<double> stow_positions)
    {
        // create a pointer that references the robot's state
        moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();
        std::vector<double> joint_group_positions;
        current_state->copyJointGroupPositions(model_group, joint_group_positions);

        // modify the joints, plan to the new joint-space goal and visualize
        for (int i = 0; i < joint_group_positions.size(); i++)
        {
            joint_group_positions[i] = stow_positions[i];
        }
        move_group_interface.setJointValueTarget(joint_group_positions);

        // Call the planner to compute the plan
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_interface.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        ROS_INFO_NAMED("tutorial", "Visualizing plan (joint-space goal) %s", success ? "" : "FAILED");

        move_group_interface.move();
    }

    // geometry_msgs::Pose getPreGraspPose(tf::StampedTransform& transform)
    // {
    //     // get the rotation (quaternion) from the transform
    //     tf::Quaternion quat = transform.getRotation();
    //     // get the yaw of the quaternion
    //     double yaw = tf::getYaw(quat);
    //     // create pre-grasp pose
    //     tf2::Quaternion pre_grasp_quat;
    //     pre_grasp_quat.setRPY(PI/2, PI/2, yaw);

    //     geometry_msgs::Pose pose;
    //     pose.orientation = tf2::toMsg(pre_grasp_quat);
    //     pose.position.x = transform.getOrigin().x();
    //     pose.position.y = transform.getOrigin().y();
    //     pose.position.z = transform.getOrigin().z() + 0.05;

    //     return pose;
    // }

    // geometry_msgs::Pose getGraspPose(tf::StampedTransform& transform)
    // {
    //     // get the rotation (quaternion) from the transform
    //     tf::Quaternion quat = transform.getRotation();
    //     // get the yaw of the quaternion
    //     double yaw = tf::getYaw(quat);
    //     // create pre-grasp pose
    //     tf2::Quaternion pre_grasp_quat;
    //     pre_grasp_quat.setRPY(PI/2, PI/2, yaw);

    //     geometry_msgs::Pose pose;
    //     pose.orientation = tf2::toMsg(pre_grasp_quat);
    //     pose.position.x = transform.getOrigin().x();
    //     pose.position.y = transform.getOrigin().y();
    //     pose.position.z = transform.getOrigin().z() - 0.01;

    //     return pose;
    // }
}