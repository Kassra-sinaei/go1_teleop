#include "rclcpp/rclcpp.hpp"
#include "position_command.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PositionCommand>("position_cmd_srv"));
    rclcpp::shutdown();

    // auto node = rclcpp::Node::make_shared("position_cmd_srv");
    // PositionCommand pos_obj = PositionCommand(node);
    // rclcpp::WallRate loop_rate(500);
}