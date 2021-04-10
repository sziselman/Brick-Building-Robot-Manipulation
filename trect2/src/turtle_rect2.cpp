/// \file turtle_rect2.cpp
/// \brief contains a node called turtle_rect2 which will make the turtle simulator move in a rectangular trajectory
///
/// PARAMETERS:
///     max_xdot (double) : the maximum linear velocity
///     max_wdot (double) : the maximum angular velocity
///     frequency (int) : the frequency of the control loop
/// PUBLISHES:
///     turtle/pose (turtlesim/msg/Pose) : the x, y, theta, linear velocity and angular velocity for the turtlesim.
/// SUBSCRIBES:
///     turtle1/cmd_vel (geometry_msgs/msg/Twist): The linear and angular command velocity for the turtlesim.
/// 
/// SERVICES:
///     trect2/start (trect2/Start): Clears the background of the turtle simulator
///     draws the desired trajectory in yellow and causes the robot to follow the path
///     (path in lavender), provides the location and dimensions of the rectangle to the node.

#include "rclcpp/rclcpp.hpp"

#include <turtlesim/msg/pose.hpp>
#include <turtlesim/srv/set_pen.hpp>
#include <turtlesim/srv/teleport_relative.hpp>
#include <turtlesim/srv/teleport_absolute.hpp>
#include <turtlesim/msg/color.hpp>
#include <std_srvs/srv/empty.hpp>

#include <geometry_msgs/msg/twist.hpp>

int main(int argc, char * argv[])
{
    /************
     * Initialize the node
     * *********/
    rclpp::init(argc, argv);
    rclcpp::Node("trect2");

    /************
     * Initialize local variables
     * *********/
    double max_xdot, max_wdot, linVel, angVel;
}
