from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch the "ros2_udp" node with the input argument "highlevel"
        Node(
            package='unitree_legged_real',
            executable='ros2_udp',
            name='udp_highlevel',
            output='screen',
            emulate_tty=True,
            arguments=['highlevel']
        ),

        # Launch the "position_srv" node from the "teleop" package
        Node(
            package='teleop',
            executable='position_server',
            name='position_srv',
            output='screen',
            emulate_tty=True
        ),
    ])
