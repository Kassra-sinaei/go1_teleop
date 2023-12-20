#pragma once

#include "iostream"
#include "fstream"
#include <vector>
#include "math.h"
#include "cmath"

#include "rclcpp/rclcpp.hpp"
#include "convert.h"

#include "ros2_unitree_legged_msgs/msg/high_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/high_state.hpp"
// #include "ros2_unitree_legged_msgs/msg/low_cmd.hpp"
// #include "ros2_unitree_legged_msgs/msg/low_state.hpp"
// #include "unitree_legged_sdk/unitree_legged_sdk.h"

using namespace std;


class PositionCommand{
    public:
        PositionCommand(rclcpp::Node* node);
        ~PositionCommand();
    private:
        rclcpp::Node* nh_;
        double* current_pos_;
        double* low_bound_;
        double* high_bound_;

};