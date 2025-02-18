
# Xbox Controller and Position Control Command for Unitree Go1

This package includes the following:
- Position Controller Server for Unitree Go1 Robot (not included in unitree SDK)
- Joystick teleoperation with Xbox controller
- LLM Assistant to operate the Go1 robot by ROSA integration. [NASA JPL ROSA GitHub repo](https://github.com/nasa-jpl/ROSA)


## Build
The services required for this package are inside the srv file and we want to make them here. First navigate to CMakeLists.txt and comment lines 21, 58, 59, 60, 61, 62, 63, 64, 67, 68. Then build the package once to generate service file.
``` bash
colcon build --packages-select teleop
```
When finished uncoment all the lines commented earlier and build using the same terminal command to build nodes.

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
## LLM Assistant
Using this node you can operate the HighLevel controller of the robot with basic command line chat. Note that you will need an active API key from [OpenAI](https://platform.openai.com/account/api-keys). Start the chat with:
```bash
./src/teleop/scripts/go1_rosa.py
```
Now simply chat with the assistant to learn what it can do for you with the Go1 robot!

## TODO

remove the open loop estimator and use the online feedback.

adjust loop duration and rate.
