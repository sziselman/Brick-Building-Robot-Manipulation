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
///     turtle_rect2/start (turtle_rect2/Start): Clears the background of the turtle simulator
///     draws the desired trajectory in yellow and causes the robot to follow the path
///     (path in lavender), provides the location and dimensions of the rectangle to the node.

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logging.hpp"
#include "rcutils/logging_macros.h"

#include "trect2/srv/start.hpp"

#include <turtlesim/msg/pose.hpp>

#include <turtlesim/srv/set_pen.hpp>
#include <turtlesim/srv/teleport_relative.hpp>
#include <turtlesim/srv/teleport_absolute.hpp>
#include <turtlesim/msg/color.hpp>

#include <std_srvs/srv/empty.hpp>

#include <geometry_msgs/msg/twist.hpp>


class turtle_rect2 : public rclcpp::Node
{
    public:
        turtle_rect2() : Node("turtle_rect2")
        {
            using std::placeholders::_1;
            using namespace std::chrono_literals;


            posePub = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
            twistSub = this->create_subscription<turtlesim::msg::Pose>("turtle1/pose", 10, std::bind(&turtle_rect2::poseCallback, this, _1));
            
            auto start = [this](const std::shared_ptr<trect2::srv::Start::Request> req, std::shared_ptr<trect2::srv::Start::Response> res) -> void {
                x = req->x;
                y = req->y;
                width = req->width;
                height = req->height;
            };

            startServ = this->create_service<trect2::srv::Start>("start", start);
            
            this->declare_parameter("max_xdot");
            this->declare_parameter("max_wdot");
            this->declare_parameter("frequency");

            this->get_parameter("max_xdot", max_xdot);
            this->get_parameter("max_wdot", max_wdot);
            this->get_parameter("frequency", frequency);

            timer = this->create_wall_timer(500ms, std::bind(&turtle_rect2::timer_callback, this));
        }

    private:

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr posePub;
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr twistSub;
        rclcpp::Service<trect2::srv::Start>::SharedPtr startServ;
        rclcpp::TimerBase::SharedPtr timer;

        double max_xdot, max_wdot;
        double width, height;
        double x, y;
        int frequency;

        turtlesim::msg::Pose turtle_pose;
        geometry_msgs::msg::Twist twist_msg;

        enum State {idle, bottom, top, left, right, rotate};
        State currentState, previousState;

        void poseCallback( turtlesim::msg::Pose::SharedPtr pose) 
        {
            turtle_pose = *pose;
        }

        void timer_callback()
        {
            twist_msg.linear.x = 1.0;
            twist_msg.angular.z = 0;
            posePub->publish(twist_msg);
        }

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<turtle_rect2>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}