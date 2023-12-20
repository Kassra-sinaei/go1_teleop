#pragma once

#include "iostream"
#include "fstream"
#include <vector>
#include "math.h"
#include "cmath"

#include "rclcpp/rclcpp.hpp"

#include "ros2_unitree_legged_msgs/msg/high_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/high_state.hpp"

#include "ros2_unitree_legged_msgs/srv/pos_cmd.hpp"

using namespace std;


class PositionCommand : public rclcpp::Node{
    public:
        PositionCommand();
        ~PositionCommand();
    private:
        double* current_pos_;
        double* low_bound_;
        double* high_bound_;
        rclcpp::Service<ros2_unitree_legged_msgs::srv::PosCmd>::SharedPtr srv_;
        rclcpp::Publisher<ros2_unitree_legged_msgs::msg::HighCmd>::SharedPtr pos_msg_;

        void send2UDP(const std::shared_ptr<ros2_unitree_legged_msgs::srv::PosCmd::Request> req,
          std::shared_ptr<ros2_unitree_legged_msgs::srv::PosCmd::Response>      res);
};