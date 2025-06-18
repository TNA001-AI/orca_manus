#!/usr/bin/env python3

# Copyright (c) 2020-2022, NVIDIA CORPORATION.  All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import time


class TestROS2Bridge(Node):
    def __init__(self):
        super().__init__("test_ros2bridge")

        # Create the publisher to publish JointState messages to the "joint_command" topic
        self.publisher_ = self.create_publisher(JointState, "joint_command", 10)

        # Initialize the JointState message
        self.joint_state = JointState()

        # Define the joint names based on the current hand model
        self.joint_state.name = [
            "right_pinky_pip",
            "right_wrist",
            "right_index_abd",
            "right_index_mcp",
            "right_index_pip",
            "right_middle_abd",
            "right_middle_mcp",
            "right_middle_pip",
            "right_pinky_abd",
            "right_pinky_mcp",
            "right_ring_abd",
            "right_ring_mcp",
            "right_ring_pip",
            "right_thumb_mcp",
            "right_thumb_abd",
            "right_thumb_pip",
            "right_thumb_dip",
        ]

        num_joints = len(self.joint_state.name)

        # Set the default joint positions
        self.default_joints = np.zeros(num_joints)

        # Set min/max joint limits (±0.5 rad around default)
        self.max_joints = self.default_joints + 0.5
        self.min_joints = self.default_joints - 0.5

        # Record start time for oscillation
        self.time_start = time.time()

        # Create a periodic timer to call the callback every 50 ms
        self.timer = self.create_timer(0.05, self.timer_callback)

    def timer_callback(self):
        # Update message timestamp
        self.joint_state.header.stamp = self.get_clock().now().to_msg()

        # Compute joint positions using sine wave oscillation
        joint_position = (
            np.sin(time.time() - self.time_start) * 0.5 * (self.max_joints - self.min_joints) + self.default_joints
        )

        self.joint_state.position = joint_position.tolist()

        # Publish the JointState message
        self.publisher_.publish(self.joint_state)


def main(args=None):
    rclpy.init(args=args)
    ros2_publisher = TestROS2Bridge()
    rclpy.spin(ros2_publisher)
    ros2_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
