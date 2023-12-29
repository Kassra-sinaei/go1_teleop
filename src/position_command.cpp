#include "position_command.h"

PositionCommand::PositionCommand(std::string passedNodeName):Node(passedNodeName){

    srv_ = this->create_service<ros2_unitree_legged_msgs::srv::PosCmd>("/pos_cmd", 
            std::bind(&PositionCommand::send2UDP, this, std::placeholders::_1, std::placeholders::_2 ));
    pos_msg_ = this->create_publisher<ros2_unitree_legged_msgs::msg::HighCmd>("/high_cmd", 1);

    current_pos_ << 0, 0, 0; 
    low_bound_ << 0, 0, 0; 
    high_bound_ << 0, 0, 0; 
    current_pos_[0] = 0; current_pos_[1] = 0; current_pos_[2] = 0; 
    // TODO: set lower bounds from input arguments
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Position Command Server Running");
}

PositionCommand::~PositionCommand(){
    
    std::cout << "Position Command Server Stopped \n";
}

void PositionCommand::send2UDP(const std::shared_ptr<ros2_unitree_legged_msgs::srv::PosCmd::Request> req,
          std::shared_ptr<ros2_unitree_legged_msgs::srv::PosCmd::Response> res){
    // check here for reference: https://qiita.com/coder-penguin/items/63f1fc1da8958f46dadd
    ros2_unitree_legged_msgs::msg::HighCmd high_cmd_ros;
    high_cmd_ros.head[0] = 0xFE;
    high_cmd_ros.head[1] = 0xEF;
    // high_cmd_ros.level_flag = HIGHLEVEL;
    high_cmd_ros.mode = 3;
    high_cmd_ros.gait_type = 2;
    high_cmd_ros.speed_level = 1;   // Between 0-2
    high_cmd_ros.foot_raise_height = 0.1;
    high_cmd_ros.body_height = 0.4;
    high_cmd_ros.euler[0] = req->phi;
    high_cmd_ros.euler[1] = 0;
    high_cmd_ros.euler[2] = 0;
    high_cmd_ros.position[0] = req->x;
    high_cmd_ros.position[1] = req->y;
    high_cmd_ros.reserve = 0;

    pos_msg_->publish(high_cmd_ros);
    Eigen::Vector3d temp(req->x, req->y, req->phi);
    this->current_pos_ = this->current_pos_ + temp;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Position Command sent...");

    res->res = true;

}