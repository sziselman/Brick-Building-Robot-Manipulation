#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class AdroitControls : public rclcpp::Node
{
    public:
    private:
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AdroitControls>());
    rclcpp::shutdown();
    return 0;
}