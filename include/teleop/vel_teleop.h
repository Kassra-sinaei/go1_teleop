#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/joy.hpp"
#include "teleop_twist_joy/teleop_twist_joy_export.h"
#include "ros2_unitree_legged_msgs/msg/high_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/high_state.hpp"
#include "unitree_legged_sdk/unitree_legged_sdk.h"

using namespace std;

class VelTeleop : public rclcpp::Node
{
    public:
        VelTeleop();
        ~VelTeleop();
    private:
        unsigned int mode_;
        unsigned int gait_type_;
        double vel_scale_;
        rclcpp::Publisher<ros2_unitree_legged_msgs::msg::HighCmd>::SharedPtr high_cmd_pub_;
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
        void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
};
