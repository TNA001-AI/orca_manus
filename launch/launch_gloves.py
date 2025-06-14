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

        )
    ])