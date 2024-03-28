
# Xbox Controller and Position COntrol Command for Unitree Go1

***

## Xbox controller

When connected to rhe robot via udp, we can no longer control it using the Unitree Joystick. Instead we can use the SDK and ROS package to move the robot while sending/receiving data from UDP.

``` bash
colcon build
cd src/teleop/launch
ros2 launch teleop_launch.py 
```

This launch file starts ros2_udp server from unitree_legged_real package, and teleop_twist_joy node.
| Controller Key/Axes | Function |
|----------|----------|
| Button A    | Activate Velocity Control Mode    |
| Button B    | Stand Down    |
| Button X    | Stand Up    |
| Left Arrow    | Trot Walk    |
| Right Arrow    | Stair Climbing    |
| Up Arrow    | Trot Run    |
| Down Arrow    | Trot Obstacle    |
| Dance 1    | LT    |
| Dance 2    | RT    |

## Position Controlled Mode

Creating a ROS2 server using the unitree_legged_real package in this [repo](https://github.com/unitreerobotics/unitree_ros2_to_real) that can respond to position commands.
First launch the server and UDP:

```bash
colcon build
cd src/teleop/launch
ros2 launch position_command.py 
```

then call in the terminal or from a client:

``` bash
ros2 service call /pos_cmd ros2_unitree_legged_msgs/srv/PosCmd "{x: 0.0, y: 0.0, phi: 0.0}"
```

## TODO

remove the open loop estimator and use the online feedback.

adjust loop duration and rate.
