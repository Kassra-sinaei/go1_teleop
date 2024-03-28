#include "position_command.h"

PositionCommand::PositionCommand(std::string passedNodeName, const rclcpp::Executor::SharedPtr& executor):
    Node(passedNodeName), executor_(executor)
    {
    // Create Call Back Groups
    cbg_service_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    cbg_subscriber_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    // Create Position Command Server
    srv_ = this->create_service<teleop::srv::PosCmd>("/pos_cmd", 
            std::bind(&PositionCommand::send2UDP, this, std::placeholders::_1, std::placeholders::_2 ),
            rmw_qos_profile_services_default, cbg_service_);
    // Initialize UDP Publisher
    pos_msg_ = this->create_publisher<ros2_unitree_legged_msgs::msg::HighCmd>("/high_cmd", 1);
    // Initialize UDP feedback
    rclcpp::SubscriptionOptions options;
    options.callback_group = cbg_service_;
    pos_estimate_ = this->create_subscription<ros2_unitree_legged_msgs::msg::HighState>("/high_state", 
                                            10, std::bind(&PositionCommand::updatePos, this, std::placeholders::_1), options);
    // Initilize executor and timer
    loop_rate_ = rclcpp::WallRate::make_shared(50);
    executor_->add_node(this->get_node_base_interface());

    current_pos_ << 0, 0, 0; 
    ol_estimate_ << 0, 0, 0; 
    current_vel_ << 0, 0, 0;
    low_bound_ << -1, -2, 0; 
    high_bound_ << 2, 2, 1; 
    // PID variables
    error_i_ << 0, 0, 0;
    error_p_ << 0, 0, 0;

    log_file_ = std::ofstream();
    log_file_.open("log_pos_ctr.csv");

    // TODO: set lower bounds from input arguments
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Position Command Server Running");
}

PositionCommand::~PositionCommand(){
    log_file_.close();
    std::cout << "Position Command Server Stopped \n";
}

void PositionCommand::run() {
    executor_->spin();
  }

void PositionCommand::updatePos(const ros2_unitree_legged_msgs::msg::HighState::SharedPtr msg){
    current_pos_ << msg->position[0], msg->position[1], msg->position[2];
    //cout << "High State Updated" << endl;
}

void PositionCommand::send2UDP(const std::shared_ptr<teleop::srv::PosCmd::Request> req,
          std::shared_ptr<teleop::srv::PosCmd::Response> res){
    Eigen::Vector3d vel;
    Eigen::Vector3d set(req->x, req->y, current_pos_(2));
    // check here for reference: https://qiita.com/coder-penguin/items/63f1fc1da8958f46dadd
    ros2_unitree_legged_msgs::msg::HighCmd high_cmd_ros;
    high_cmd_ros.head[0] = 0xFE;
    high_cmd_ros.head[1] = 0xEF;
    high_cmd_ros.level_flag = UNITREE_LEGGED_SDK::HIGHLEVEL;
    high_cmd_ros.mode = 2;
    high_cmd_ros.gait_type = 1;
    high_cmd_ros.speed_level = 0;   // Between 0-2
    high_cmd_ros.foot_raise_height = 0.1;
    high_cmd_ros.body_height = 0.1;
    //high_cmd_ros.euler[0] = req->phi;
    high_cmd_ros.euler[0] = 0;
    high_cmd_ros.euler[1] = 0;
    high_cmd_ros.euler[2] = 0;
    high_cmd_ros.reserve = 0;

    double time = 0;
    //chrono::milliseconds max_iter = chrono::milliseconds(2);	
    while(time < 4 * 50)    // modify this duration
    {
        time+=1;
        // this->pidController(vel, set, this->current_pos_);
        this->pidController(vel, set, ol_estimate_);
        log_file_ << vel(0) << "," << vel(1) << "," << vel(2) << ",";
        log_file_ << current_pos_(0) << "," << current_pos_(1) << "," << current_pos_(2) << "\n"; 
        this->clamp(low_bound_, high_bound_, vel);
        high_cmd_ros.velocity[0] = vel(0);
        high_cmd_ros.velocity[1] = vel(1);
        pos_msg_->publish(high_cmd_ros);
        //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "I'm fine");
        loop_rate_->sleep();
        ol_estimate_ = ol_estimate_ + vel / 50;
        //executor_->spin_some(max_iter);
    }
    // high_cmd_ros.velocity[0] = 0;
    // high_cmd_ros.velocity[1] = 0;
    // high_cmd_ros.mode = 0;
    // pos_msg_->publish(high_cmd_ros);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Position Command sent...");
    //double error_norm = (set - current_pos_).norm();
    double error_norm = (set - ol_estimate_).norm();
    if(error_norm < 0.05){
        res->res = true;
    }else{
        res->res = false;
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Position Error more than threshold.");
        cout << "Position Error norm (m):" << error_norm << endl;;
    }

}

void PositionCommand::pidController(Eigen::Vector3d &res, Eigen::Vector3d set_point, Eigen::Vector3d current_state, 
                            double kp, double kd, double ki){
    Eigen::Vector3d error = set_point - current_state;
    error_i_ = error_i_ + error;
    res = kp * error + ki * error_i_ + kd * error_p_;
    error_p_ = error;
}

template<typename T> void PositionCommand::clamp(T low, T high, T &input){
    input << min(max(input(0), low(0)), high(0)),
              min(max(input(1), low(1)), high(1)),
              min(max(input(2), low(2)), high(2));
}