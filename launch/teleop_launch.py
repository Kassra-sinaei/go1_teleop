from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='unitree_legged_real',
            executable='ros2_udp',  # replace with your executable name
            arguments=['highlevel'],
            output='screen'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('teleop_twist_joy'), '/launch/teleop-launch.py']),
            launch_arguments={'joy_config': 'xbox'}.items(),
        ),
        Node(
            package='teleop',  # replace with your package name
            executable='joy_stick_teleop',  # replace with your executable name
            output='screen'
        ),
    ])