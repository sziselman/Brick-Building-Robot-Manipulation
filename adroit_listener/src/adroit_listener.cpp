#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

using std::placeholders::_1;

class AdroitSubscriber : public rclcpp::Node
{
    public:
        AdroitSubscriber() : Node("adroit_listener_node")
        {
            sub = this->create_subscription<std_msgs::msg::Bool>("manipulator_connected", 10, std::bind(&AdroitSubscriber::manipulator_callback, this, _1));
        }

    private:

        void manipulator_callback(const std_msgs::msg::Bool::SharedPtr msg) const
        {
            if (msg->data == true)
            {
                RCLCPP_INFO(this->get_logger(), "The xbox controller is connected!");
            } else if (msg->data == false)
            {
                RCLCPP_INFO(this->get_logger(), "The xbox controller is not connected!");
            } else
            {
                RCLCPP_INFO(this->get_logger(), "The subscriber is not receiving anything!");
            }
        }

        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AdroitSubscriber>());
    rclcpp::shutdown();
    return 0;
}