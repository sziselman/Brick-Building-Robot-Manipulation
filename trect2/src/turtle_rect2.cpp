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

#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "trect2/srv/start.hpp"

#include <turtlesim/msg/pose.hpp>

#include <turtlesim/srv/set_pen.hpp>
#include <turtlesim/srv/teleport_relative.hpp>
#include <turtlesim/srv/teleport_absolute.hpp>
#include <turtlesim/msg/color.hpp>

#include <std_srvs/srv/empty.hpp>

#include <geometry_msgs/msg/twist.hpp>


using std::placeholders::_1;
using namespace std::chrono_literals;

class trect2 : public rclcpp::Node
{
    public:
        trect2() : Node("trect2")
        {
            posePub_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
            twistSub_ = this->create_subscription<turtlesim::msg::Pose>("turtle1/pose", 10, std::bind(&trect2::poseCallback, this, _1));
            startServ_ = this->create_service<trect2::srv::start>("/start", &start);
            
            this->declare_parameter("max_xdot");
            this->declare_parameter("max_wdot");
            this->declare_parameter("frequency");

            this->get_parameter("max_xdot", max_xdot);
            this->get_parameter("max_wdot", max_wdot);
            this->get_parameter("frequency", frequency);

            timer_ = this->create_wall_timer(500ms, std::bind(&trect2::timer_callback, this));
        }

    private:
        double max_xdot, max_wdot, width, height;
        int frequency;
        turtlesim::msg::Pose turtle_pose;
        geometry_msgs::msg::Twist twist_msg;

        enum State {Idle, bottom, top, left, right, Rotate};
        State currentState, previousState;

        void poseCallback( turtlesim::msg::Pose::SharedPtr pose) 
        {
            turtle_pose = *pose;
        }

        void timer_callback()
        {
            twist_msg.linear.x = 1.0;
            twist_msg.angular.z = 0;
            posePub_->publish(twist_msg);
        }

        void start(const std::shared_ptr<trect2::srv::start::Request> req, std::shared_ptr<trect2::srv::start::Response> res)
        {
            width = req.width;
            height = req.height;

        }
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr posePub_;
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr twistSub_;
        rclcpp::Service<trect2::srv::start>::SharedPtr startServ_;
        rclcpp::TimerBase::SharedPtr timer_;
        
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<trect2>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}