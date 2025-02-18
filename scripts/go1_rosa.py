#!/usr/bin/env python3

from langchain.agents import tool, Tool
from rosa import ROSA, RobotSystemPrompts
from langchain_openai import ChatOpenAI
import os

from rclpy import spin_until_future_complete
from ros2_unitree_legged_msgs.srv import PosCmd
import rclpy
from ros2_unitree_legged_msgs.msg import HighCmd



class Go1MotionClient(ROSA):

    def __init__(self, streaming:bool=False):
        ## ROSA Initialization
        self.__blacklist = ["master", "docker"]
        self.__prompts = RobotSystemPrompts(
        embodiment_and_persona="You are a ros2 service caller to move a unitree go1 robot.",
        about_your_operators="Your operators are robotics engineer who want to experiment with go1 robot.",
        critical_instructions="You should always inform the operator of the result of service calls.",
        about_your_environment="The robot is inside a univerity lab on a flat ground.",
        about_your_capabilities="You can call services to move the robot inside lab and does a few simple dances."
        )
        self.__llm = ChatOpenAI(model="gpt-3.5-turbo", api_key=os.environ.get('OPENAI_API_KEY'))
        self.__streaming = streaming
        # Tools for agent
        move_forward = Tool(
            name="move_forward",
            description="Move the robot forward by the specified distance.",
            func=self.move_forward
            )
        
        stand_up = Tool(
            name="stand_up",
            description="Stand up the robot from sitting position.",
            func=self.stand_up
        )

        sit_down = Tool(
            name="sit_down",
            description="Sit down the robot in resting posture.",
            func=self.sit
        )
        super().__init__(
            ros_version=2, llm=self.__llm, 
            tools=[move_forward, stand_up, sit_down], 
            prompts=self.__prompts, streaming=self.__streaming, blacklist=self.__blacklist, verbose=True, accumulate_chat_history=True)
        
        ## ROS2 Node Setup
        self.node_ = rclpy.create_node('go1_motion_client')
        self.cli_ = self.node_.create_client(PosCmd, '/pos_cmd')
        self.high_cmd_pub_ = self.node_.create_publisher(HighCmd, '/high_cmd', 1)
        while not self.cli_.wait_for_service(timeout_sec=1.0):
            self.node_.get_logger().info('service not available, waiting again...')
        self.req_ = PosCmd.Request()
        self.send_request(x=0, y=0, phi=0)


    def send_request(self, x: float = 0, y: float = 0, phi: float = 0):
        self.req_.x = float(x)
        self.req_.y = float(y)
        self.req_.phi = float(phi)
        self.future = self.cli_.call_async(self.req_)
        # spin_until_future_complete(self, self.future)
        return self.future.result()
    
    def move_forward(self, distance: float) -> str:
        """
        Move the robot forward by the specified distance.
        
        :param distance: The distance to move the robot forward.
        """
        self.send_request(x=distance)
        return f"Moving forward by {distance} units."
    
    def stand_up(self, input = None) -> str:
        """
        Stand up the robot from sitting position.
        """
        msg = HighCmd()
        msg.head[0] = 0xFE
        msg.head[1] = 0xEF
        msg.speed_level = 1
        msg.foot_raise_height = 0.08
        msg.body_height = 0.28
        msg.velocity[0] = float(0.0)
        msg.velocity[1] = float(0.0)
        msg.yaw_speed = float(0.0)
        msg.mode = 6
        self.high_cmd_pub_.publish(msg)
        return "Standing up"
    
    def  sit(self, input = None) -> str:
        """
        Stand Down the robot in its resting posture.
        """
        msg = HighCmd()
        msg.head[0] = 0xFE
        msg.head[1] = 0xEF
        msg.speed_level = 1
        msg.foot_raise_height = 0.08
        msg.body_height = 0.28
        msg.velocity[0] = float(0.0)
        msg.velocity[1] = float(0.0)
        msg.yaw_speed = float(0.0)
        msg.mode = 5
        self.high_cmd_pub_.publish(msg)
        return "Standing Down"
    
    def dance(self) -> str:
        """
        Make the robot do a simple dance.
        """
        msg = HighCmd()
        msg.head[0] = 0xFE
        msg.head[1] = 0xEF
        msg.speed_level = 1
        msg.foot_raise_height = 0.08
        msg.body_height = 0.28
        msg.velocity[0] = float(0.0)
        msg.velocity[1] = float(0.0)
        msg.yaw_speed = float(0.0)
        msg.mode = 12
        self.high_cmd_pub_.publish(msg)
        return "Dancing"
    
    def run(self):
        print("Agent online. Write your prompts to work with Go1 robot.")
        print("Type 'exit' to quit.")
        while True:
            user_input = input("Enter command (or 'exit' to quit): ")
            if user_input.lower() == 'exit':
                print("Exiting...")
                break
            else:
                response = self.invoke(user_input)
                print(response)
    
    def destroy_node(self):
        self.node_.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Go1MotionClient()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()