/*! @package interfaces_node
    Code Information:
        Maintainer: Eng. Davidson Daniel Rojas Cediel
        Mail: davidson@kiwibot.com
        Kiwi Campus / AI & Robotics Team
*/

#include "modules/speaker.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    rclcpp::executors::SingleThreadedExecutor executor;

    auto speaker_node = std::make_shared<Speaker>(options);
    executor.add_node(speaker_node);
    auto period = std::chrono::milliseconds(100);
    rclcpp::Rate r(period);
    while (rclcpp::ok())
    {
        executor.spin_some();
        r.sleep();
    }
    rclcpp::shutdown();

    return 0;
}