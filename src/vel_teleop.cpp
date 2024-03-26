#include "vel_teleop.h"

VelTeleop::VelTeleop() : Node("vel_teleop")
{
    high_cmd_pub_ = this->create_publisher<ros2_unitree_legged_msgs::msg::HighCmd>("/high_cmd", 1);
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, std::bind(&VelTeleop::joyCallback, this, std::placeholders::_1));

    mode_ = 0;          // Default mode is idle
    gait_type_ = 1;     // Default gait type is trot walk
    vel_scale_ = 0.5;
}

VelTeleop::~VelTeleop()
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Vel Teleop Node Stopped");
}

void VelTeleop::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    ros2_unitree_legged_msgs::msg::HighCmd high_cmd_ros;
    high_cmd_ros.head[0] = 0xFE;
    high_cmd_ros.head[1] = 0xEF;
    high_cmd_ros.level_flag = UNITREE_LEGGED_SDK::HIGHLEVEL;
    high_cmd_ros.speed_level = 1;   // Between 0-2
    high_cmd_ros.foot_raise_height = 0.08;
    high_cmd_ros.body_height = 0.28;
    high_cmd_ros.velocity[0] = 0.0;
    high_cmd_ros.velocity[1] = 0.0;
    high_cmd_ros.yaw_speed = 0.0;

    if (msg->buttons[4] == 1){
        mode_ = 0;  // Idle
    } else if (msg->buttons[0] == 1){
        mode_ = 2;      // Velocity control walk
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Mode -> Velocity Mode");
    } else if (msg->buttons[1] == 1){
        mode_ = 5;      // Stand Down
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Mode -> Standing Down");
    } else if (msg->buttons[3] == 1){
        mode_ = 6;      // Stand Up
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Mode -> Standing Up");
    } else if (msg->buttons[6] == 1){
        mode_ = 12;      // Dance 1
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Mode -> Dance 1");
    }else if (msg->buttons[7] == 1){
        mode_ = 13;      // Dance 2
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Mode -> Dance 2");
    }

    if (mode_ == 2 && msg->axes[6] == 1){
        gait_type_ = 1;     // Trot walk
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Gait Type -> Trot Walk");
    } else if (mode_ == 2 && msg->axes[7] == 1){
        gait_type_ = 2;     // Trot Run
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Gait Type -> Trot Run");
    } else if (mode_ == 2 && msg->axes[6] == -1){
        gait_type_ = 3;     // Stair Climbing
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Gait Type -> Stair Climbing");
    } else if (mode_ == 2 && msg->axes[7] == -1){
        gait_type_ = 4;     // Trot Obstacle
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Gait Type -> Trot Obstacle");
    }

    if (mode_ == 2){
        high_cmd_ros.velocity[0] = vel_scale_ * msg->axes[1];
        high_cmd_ros.velocity[1] = vel_scale_ * msg->axes[0];
        high_cmd_ros.yaw_speed = vel_scale_ * msg->axes[2];
    }

    high_cmd_ros.mode = mode_;
    high_cmd_ros.gait_type = gait_type_;
    high_cmd_ros.reserve = 0;
    high_cmd_pub_->publish(high_cmd_ros);
}