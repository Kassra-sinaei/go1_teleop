# Position Controller ROS2 Server for Go1 
***
## Description
Creating a ROS2 server using the unitree_legged_real package in this [repo](https://github.com/unitreerobotics/unitree_ros2_to_real) that can respond to position commands.
First launch the server and UDP:
```
$ colcon build
$ cd src/teleop/launch
$ ros2 launch position_command.py 
```
then call in the terminal or from a client:
```
$ ros2 service call /pos_cmd ros2_unitree_legged_msgs/srv/PosCmd "{x: 0.0, y: 0.0, phi: 0.0}"
```
## TODO
remove the open loop estimator and use the online feedback.

adjust loop duration and rate.
