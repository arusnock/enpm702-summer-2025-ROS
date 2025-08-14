#include <rclcpp/rclcpp.hpp>
#include "bot_controller_demo/proportional_controller.hpp"


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto controller = std::make_shared<ProportionalController>();

    try {
        rclcpp::spin(controller);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(controller->get_logger(), "Exception caught: %s", e.what());
    } catch (...) {
        RCLCPP_ERROR(controller->get_logger(), "Unknown exception caught");
    }

    // Stop robot before shutdown
    controller->stop_robot();
    rclcpp::shutdown();
}