#include "multi_lidar_calibrator.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ROSMultiLidarCalibratorApp>();

    RCLCPP_INFO(node->get_logger(), "Node started");

    rclcpp::spin(node);

    rclcpp::shutdown();

    RCLCPP_INFO(node->get_logger(), "Node shutdown");

    return 0;
}
