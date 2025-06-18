from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='isaac_sim',
            executable='joint_converter',
            name='joint_converter',
            output='screen',
            parameters=[],
        ),
    ]) 