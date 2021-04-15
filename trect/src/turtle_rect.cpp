#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logging.hpp"
#include "rcutils/logging_macros.h"

#include <turtlesim/msg/pose.hpp>

#include <turtlesim/srv/set_pen.hpp>
#include <turtlesim/srv/teleport_relative.hpp>
#include <turtlesim/srv/teleport_absolute.hpp>
#include <turtlesim/msg/color.hpp>

#include "trect/srv/start.hpp"

#include <std_srvs/srv/empty.hpp>

#include <geometry_msgs/msg/twist.hpp>

class turtle_rect : public rclcpp::Node
{
    public:
        turtle_rect() : Node("turtle_rect")
        {
            using std::placeholders::_1;
            using namespace std::chrono_literals;

            posePub = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
            twistSub = this->create_subscription<turtlesim::msg::Pose>("turtle1/pose", 10, std::bind(&turtle_rect::poseCallback, this, _1));

            auto start = [this](const std::shared_ptr<trect::srv::Start::Request> req, std::shared_ptr<trect::srv::Start::Response> res) -> void {
                x = req->x;
                y = req->y;
                width = req->width;
                height = req->height;

                /*******
                 * Set color of the pen to pastel yellow
                 * ****/
                auto pen_color = std::make_shared<turtlesim::srv::SetPen::Request>();

                pen_color->r = 253;
                pen_color->b = 150;
                pen_color->g = 253;
                pen_color->width = 5;
                pen_color->off = 0;

                pen_client->async_send_request(pen_color);

                /*******
                 * Clear background of turtle simulator
                 * ****/
                auto empty = std::make_shared<std_srvs::srv::Empty::Request>();

                empty_client->async_send_request(empty);

                /*******
                 * Move turtle to designated start position
                 * ****/
                auto absPos = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
                absPos->x = x;
                absPos->y = y;
                absPos->theta = 0;
                teleAbs_client->async_send_request(absPos);
                
                empty_client->async_send_request(empty);

                /*******
                 * Have turtle draw the rectangle in yellow
                 * ****/
                absPos->x = x + width;
                teleAbs_client->async_send_request(absPos);

                absPos->y = y + height;
                teleAbs_client->async_send_request(absPos);

                absPos->x = x;
                teleAbs_client->async_send_request(absPos);

                absPos->y = y;
                teleAbs_client->async_send_request(absPos);

                /*******
                 * Change color of the pen to lavender
                 * ****/
                pen_color->r = 182;
                pen_color->b = 104;
                pen_color->g = 182;

                pen_client->async_send_request(pen_color);

                /********
                 * Put turtle in bottom state
                 * *****/
                currState = bottom;

            };

            startServ = this->create_service<trect::srv::Start>("start", start);
            startClient = this->create_client<trect::srv::Start>("start");
            pen_client = this->create_client<turtlesim::srv::SetPen>("turtle1/set_pen");
            teleAbs_client = this->create_client<turtlesim::srv::TeleportAbsolute>("turtle1/teleport_absolute");
            teleRel_client = this->create_client<turtlesim::srv::TeleportRelative>("turtle1/teleport_relative");
            empty_client = this->create_client<std_srvs::srv::Empty>("/clear");

            this->declare_parameter("max_xdot");
            this->declare_parameter("max_wdot");
            this->declare_parameter("frequency");

            this->get_parameter("max_xdot", max_xdot);
            this->get_parameter("max_wdot", max_wdot);
            this->get_parameter("frequency", frequency);

            timer = this->create_wall_timer(500ms, std::bind(&turtle_rect::timer_callback, this));
        }


    private:

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr posePub;
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr twistSub;
        rclcpp::Service<trect::srv::Start>::SharedPtr startServ;
        rclcpp::Client<trect::srv::Start>::SharedPtr startClient;
        rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr pen_client;
        rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedPtr teleAbs_client;
        rclcpp::Client<turtlesim::srv::TeleportRelative>::SharedPtr teleRel_client;
        rclcpp::Client<std_srvs::srv::Empty>::SharedPtr empty_client;

        rclcpp::TimerBase::SharedPtr timer;

        double max_xdot, max_wdot;
        double width, height;
        double x, y;
        int frequency;

        turtlesim::msg::Pose turtle_pose;
        geometry_msgs::msg::Twist twist_msg;

        enum State {idle, bottom, top, left, right, rotate};
        State currState = idle;
        State prevState = idle;

        void poseCallback( turtlesim::msg::Pose::SharedPtr pose) 
        {
            turtle_pose = *pose;
        }

        void timer_callback()
        {
            switch (currState)
            {
                /*******
                 * Idle: Turtle is idle before moving in rectangular trajectory and after finishing moving in rectangle
                 * ****/
                case idle:
                    break;
                case bottom:
                    break;
                case right:
                    break;
                case top:
                    break;
                case left:
                    break;
                case rotate:
                    break;
            }
        }

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<turtle_rect>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}