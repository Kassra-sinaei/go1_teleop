#include "position_command.h"

PositionCommand::PositionCommand(rclcpp::Node* node){
    nh_ = node;
    std::cout << "Position Command Server Running \n";
}

PositionCommand::~PositionCommand(){
    delete[] low_bound_;
    delete[] high_bound_;
    std::cout << "Position Command Server Stopped \n";
}