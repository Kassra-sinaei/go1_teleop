#pragma once

#include "fstream"
#include "eigen3/Eigen/Eigen"
#include "math.h"
#include "cmath"
#include "string.h"

#include "rclcpp/rclcpp.hpp"
#include "chrono"

#include "ros2_unitree_legged_msgs/msg/high_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/high_state.hpp"
#include "unitree_legged_sdk/unitree_legged_sdk.h"

#include "ros2_unitree_legged_msgs/srv/pos_cmd.hpp"

using namespace std;


class PositionCommand : public rclcpp::Node{
    public:
        PositionCommand(std::string passedNodeName);
        ~PositionCommand();
    private:
        Eigen::Vector3d current_pos_;
        Eigen::Vector3d low_bound_;
        Eigen::Vector3d high_bound_;
        rclcpp::Service<ros2_unitree_legged_msgs::srv::PosCmd>::SharedPtr srv_;
        rclcpp::Publisher<ros2_unitree_legged_msgs::msg::HighCmd>::SharedPtr pos_msg_;
        
        rclcpp::WallRate::SharedPtr loop_rate_;
        //rclcpp::executors::SingleThreadedExecutor* executor_;

        void send2UDP(const std::shared_ptr<ros2_unitree_legged_msgs::srv::PosCmd::Request> req,
          std::shared_ptr<ros2_unitree_legged_msgs::srv::PosCmd::Response> res);
};