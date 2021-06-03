#include "control/control_library.hpp"

namespace control_library
{
    void moveArm(moveit::planning_interface::MoveGroupInterface& move_group, geometry_msgs::Pose& pose)
    {
        move_group.setPoseTarget(pose);

        // Call the planner to compute the plan and visualize it
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        ROS_INFO_NAMED("tutorial", "Visualizing plan (pose goal) %s", success ? "" : "FAILED");

        // moving to pose goal
        move_group.move();
    }

    void movePincer(moveit::planning_interface::MoveGroupInterface& move_group, const moveit::core::JointModelGroup* model_group, double joint_angle)
    {
        // a pointer that references the current robot's states
        moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
        // get the current set of joint values for the group
        std::vector<double> joint_group_positions;
        current_state->copyJointGroupPositions(model_group, joint_group_positions);

        // modify the first joint position
        joint_group_positions[0] =joint_angle;

        // pass the desired joint positions to move_group as goal for planning
        move_group.setJointValueTarget(joint_group_positions);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        move_group.move(); 
    }

    void stowPosition(moveit::planning_interface::MoveGroupInterface& move_group, const moveit::core::JointModelGroup* model_group, std::vector<double> stow_positions)
    {
        // create a pointer that references the robot's state
        moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
        std::vector<double> joint_group_positions;
        current_state->copyJointGroupPositions(model_group, joint_group_positions);

        // modify the joints, plan to the new joint-space goal and visualize
        for (int i = 0; i < joint_group_positions.size(); i++)
        {
            joint_group_positions[i] = stow_positions[i];
        }
        move_group.setJointValueTarget(joint_group_positions);

        // Call the planner to compute the plan
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        ROS_INFO_NAMED("tutorial", "Visualizing plan (joint-space goal) %s", success ? "" : "FAILED");

        move_group.move();
    }

    geometry_msgs::Pose getPreGraspPose(geometry_msgs::TransformStamped& transform)
    {
        // convert the msg to tf
        tf2::Quaternion quat_tf;
        tf2::fromMsg(transform.transform.rotation, quat_tf);
        
        tf2::Matrix3x3 matrix(quat_tf);

        double roll, pitch, yaw;
        matrix.getRPY(roll, pitch, yaw);

        tf2::Quaternion pre_grasp_quat;
        pre_grasp_quat.setRPY(roll + PI/2, pitch + PI/2, yaw);

        geometry_msgs::Pose pose;
        pose.orientation = tf2::toMsg(pre_grasp_quat);
        pose.position.x = transform.transform.translation.x;
        pose.position.y = transform.transform.translation.y;
        pose.position.z = transform.transform.translation.z + 0.05;

        return pose;
    }

    geometry_msgs::Pose getGraspPose(geometry_msgs::TransformStamped& transform)
    {
        // convert the msg to tf
        tf2::Quaternion quat_tf;
        tf2::fromMsg(transform.transform.rotation, quat_tf);
        
        tf2::Matrix3x3 matrix(quat_tf);

        double roll, pitch, yaw;
        matrix.getRPY(roll, pitch, yaw);

        tf2::Quaternion pre_grasp_quat;
        pre_grasp_quat.setRPY(roll + PI/2, pitch + PI/2, yaw);

        geometry_msgs::Pose pose;
        pose.orientation = tf2::toMsg(pre_grasp_quat);
        pose.position.x = transform.transform.translation.x;
        pose.position.y = transform.transform.translation.y;
        pose.position.z = transform.transform.translation.z - 0.01;

        return pose;
    }

    std::vector<geometry_msgs::Pose> buildWall(std::vector<double>& brick_dims, int num)
    {
        std::vector<geometry_msgs::Pose> brick_poses;
        brick_poses.resize(num);

        tf2::Quaternion brick_quat;
        brick_quat.setRPY(0.0, 0.0, 0.0);

        int row = 0;
        int col = 0;

        double x = 0.0;
        double y = 0.0;
        double z = brick_dims[2];

        for (int n = 0; n < num; n++)
        {
            geometry_msgs::Pose brick_pose;
            brick_pose.orientation = tf2::toMsg(brick_quat);
            brick_pose.position.x = x;
            brick_pose.position.y = y;
            brick_pose.position.z = z;

            brick_poses.push_back(brick_pose);

            y += brick_dims[1]+0.01;
        }

        return brick_poses;
    }
}