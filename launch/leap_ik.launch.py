from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='glove',
            executable='read_and_send_zmq',
            name='read_and_send_zmq',
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='telekinesis',
            executable='leap_ik',
            name='leap_ik',
            output='screen',
            emulate_tty=True,
            parameters=[
            {"isLeft": False},
            ]
        )
    ])