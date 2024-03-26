#include "vel_teleop.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VelTeleop>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}