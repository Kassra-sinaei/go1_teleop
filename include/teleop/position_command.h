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
#include "ros2_unitree_legged_msgs/msg/high_state.hpp"
#include "unitree_legged_sdk/unitree_legged_sdk.h"

#include "ros2_unitree_legged_msgs/srv/pos_cmd.hpp"

using namespace std;


class PositionCommand : public rclcpp::Node{
    public:
        PositionCommand(std::string passedNodeName, const rclcpp::Executor::SharedPtr& executor);
        ~PositionCommand();
        void run();
        template<typename T> void clamp(T low, T high, T &input);
    private:
        Eigen::Vector3d current_pos_;
        Eigen::Vector3d ol_estimate_;
        Eigen::Vector3d current_vel_;
        Eigen::Vector3d low_bound_;
        Eigen::Vector3d high_bound_;
        Eigen::Vector3d error_i_;
        Eigen::Vector3d error_p_;
        rclcpp::Service<ros2_unitree_legged_msgs::srv::PosCmd>::SharedPtr srv_;
        rclcpp::Publisher<ros2_unitree_legged_msgs::msg::HighCmd>::SharedPtr pos_msg_;
        rclcpp::Subscription<ros2_unitree_legged_msgs::msg::HighState>::SharedPtr pos_estimate_;
        
        rclcpp::WallRate::SharedPtr loop_rate_;
        rclcpp::Executor::SharedPtr executor_;
        rclcpp::CallbackGroup::SharedPtr cbg_service_ = std::make_shared<rclcpp::CallbackGroup>(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::CallbackGroup::SharedPtr cbg_subscriber_ = std::make_shared<rclcpp::CallbackGroup>(rclcpp::CallbackGroupType::MutuallyExclusive);

        std::ofstream log_file_;

        void send2UDP(const std::shared_ptr<ros2_unitree_legged_msgs::srv::PosCmd::Request> req,
          std::shared_ptr<ros2_unitree_legged_msgs::srv::PosCmd::Response> res);
        void updatePos(const ros2_unitree_legged_msgs::msg::HighState::SharedPtr msg);
        void pidController(Eigen::Vector3d &res, Eigen::Vector3d set_point, Eigen::Vector3d current_state, 
                            double kp=0.5, double kd = 0, double ki = 0);
};