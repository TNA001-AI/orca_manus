#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Manus right-hand glove  ➜  Isaac Sim joint-command bridge (index-based)

Subscribes :
    /glove/r_joints   (sensor_msgs/JointState)  -  20 positions, no names
Publishes  :
    /joint_command  (sensor_msgs/JointState)

Assumed Manus order (per finger, thumb→pinky):
    MCP_side, MCP_forward, PIP, DIP   →  5 fingers * 4 joints = 20
"""

import math
import signal
import sys
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState


# ---------- Edit to match your USD/URDF joint names ----------
ISAAC_NAMES: List[str] = [
    # Thumb (4)
    "right_thumb_mcp",   # Manus index 0
    "right_thumb_abd",   # Manus index 1
    "right_thumb_pip",   # Manus index 2
    "right_thumb_dip",   # Manus index 3
    # Index (4)
    "right_index_abd",   # 4
    "right_index_mcp",   # 5
    "right_index_pip",   # 6
    None,                # 7  (index DIP missing)
    # Middle (4)
    "right_middle_abd",  # 8
    "right_middle_mcp",  # 9
    "right_middle_pip",  # 10
    None,                # 11 (middle DIP missing)
    # Ring (4)
    "right_ring_abd",    # 12
    "right_ring_mcp",    # 13
    "right_ring_pip",    # 14
    None,                # 15 (ring DIP missing)
    # Pinky (4)
    "right_pinky_abd",   # 16
    "right_pinky_mcp",   # 17
    "right_pinky_pip",   # 18
    None,                # 19 (pinky DIP missing)
]
# -------------------------------------------------------------


class RightHandBridge(Node):
    """Forward Manus glove joint angles (degrees) to Isaac Sim (radians)."""

    def __init__(self):
        super().__init__("glove_right_bridge")

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            depth=10,
        )

        self.sub_r = self.create_subscription(
            JointState, "/glove/r_joints", self._cb_right, qos
        )
        self.pub_r = self.create_publisher(
            JointState, "joint_command", qos
        )

        self.latest: JointState | None = None
        self.timer = self.create_timer(0.05, self._timer_cb)  # 20 Hz

        self.get_logger().info("Bridge running - waiting for /glove/r_joints …")

    # ---------- Callbacks ----------

    def _cb_right(self, msg: JointState):
        """Store the most recent glove message."""
        self.latest = msg

    def _timer_cb(self):
        """Publish remapped JointState every timer tick."""
        if not self.latest:
            return

        if len(self.latest.position) < len(ISAAC_NAMES):
            self.get_logger().warn("Received position array < 20 elements - skipping frame")
            return

        # Convert Manus degrees to radians
        positions_rad = [math.radians(p) for p in self.latest.position[:20]]

        # Build JointState for existing joints only
        out = JointState()
        out.header = self.latest.header
        out.name = [n for n in ISAAC_NAMES if n is not None]
        out.position = [
            pos if name is not None else 0.0
            for pos, name in zip(positions_rad, ISAAC_NAMES)
            if name is not None
        ]
        self.pub_r.publish(out)

    # ---------- Shutdown helper ----------

    def stop_hand(self):
        """Send zero pose before shutting down."""
        zero = JointState()
        zero.name = ISAAC_NAMES
        zero.position = [0.0] * len(ISAAC_NAMES)
        self.pub_r.publish(zero)


# ---------------- main ----------------

def main(args=None):
    rclpy.init(args=args)
    node = RightHandBridge()

    def sigint_handler(sig, frame):
        node.get_logger().info("SIGINT - stopping hand.")
        node.stop_hand()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, sigint_handler)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
