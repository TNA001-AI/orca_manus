#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
import numpy as np


class JointConverter(Node):
    """
    ROS2 node that converts /joint_states messages to /joint_command_right_hand messages
    with the correct joint order mapping.
    """
    
    def __init__(self):
        super().__init__('joint_converter')
        
        # Define the input joint order (from /joint_states)
        self.input_joint_names = [
            'right_wrist',  
            'right_thumb_mcp', 'right_thumb_abd', 'right_thumb_pip', 'right_thumb_dip',
            'right_index_abd', 'right_index_mcp', 'right_index_pip',
            'right_middle_abd', 'right_middle_mcp', 'right_middle_pip',
            'right_ring_abd', 'right_ring_mcp', 'right_ring_pip',
            'right_pinky_abd', 'right_pinky_mcp', 'right_pinky_pip',
        ]
        
        # Define the output joint order (for /joint_command_right_hand)
        self.output_joint_names = [
            "right_wrist",
            "right_index_abd", "right_index_mcp", "right_index_pip",
            "right_middle_abd", "right_middle_mcp", "right_middle_pip",
            "right_pinky_abd", "right_pinky_mcp", "right_pinky_pip",
            "right_ring_abd", "right_ring_mcp", "right_ring_pip",
            "right_thumb_mcp", "right_thumb_abd", "right_thumb_pip", "right_thumb_dip",
        ]
        
        # Create mapping from input to output indices
        self.joint_mapping = {}
        for i, joint_name in enumerate(self.input_joint_names):
            if joint_name in self.output_joint_names:
                output_idx = self.output_joint_names.index(joint_name)
                self.joint_mapping[i] = output_idx
        
        self.get_logger().info(f"Joint mapping created: {self.joint_mapping}")
        
        # Create subscribers and publishers
        self.joint_states_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            qos_profile_sensor_data
        )
        
        self.joint_command_pub = self.create_publisher(
            JointState,
            '/joint_command_right_hand',
            10
        )
        
        self.get_logger().info('Joint converter node initialized')
        self.get_logger().info(f'Subscribing to: /joint_states')
        self.get_logger().info(f'Publishing to: /joint_command_right_hand')
    
    def joint_states_callback(self, msg):
        """
        Callback function that processes incoming joint_states messages
        and converts them to joint_command_right_hand format.
        """
        try:
            # Check if we have the expected number of joints
            if len(msg.name) != len(self.input_joint_names):
                self.get_logger().warn(
                    f'Expected {len(self.input_joint_names)} joints, got {len(msg.name)}'
                )
                return
            
            # Check if joint names match
            if msg.name != self.input_joint_names:
                self.get_logger().warn(
                    f'Joint names do not match expected order. Expected: {self.input_joint_names[:5]}..., Got: {msg.name[:5]}...'
                )
                return
            
            # Create output JointState message
            output_msg = JointState()
            output_msg.header = msg.header  # Copy the header from input message
            output_msg.name = self.output_joint_names
            
            # Initialize arrays with correct size
            output_msg.position = [0.0] * len(self.output_joint_names)
            # output_msg.velocity = [0.0] * len(self.output_joint_names)
            # output_msg.effort = [0.0] * len(self.output_joint_names)
            
            # Map joint positions from input to output order
            for input_idx, output_idx in self.joint_mapping.items():
                if input_idx < len(msg.position):
                    output_msg.position[output_idx] = msg.position[input_idx]
                
                # # Also map velocity and effort if available
                # if input_idx < len(msg.velocity):
                #     output_msg.velocity[output_idx] = msg.velocity[input_idx]
                
                # if input_idx < len(msg.effort):
                #     output_msg.effort[output_idx] = msg.effort[input_idx]
            
            # Publish the output message
            self.joint_command_pub.publish(output_msg)
            
            # Log some debug info occasionally
            if self.get_clock().now().nanoseconds % 1000000000 < 10000000:  # Every ~1 second
                self.get_logger().debug(
                    f'Converted {len(msg.position)} joints to {len(output_msg.position)} joints'
                )
                
        except Exception as e:
            self.get_logger().error(f'Error in joint_states_callback: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    node = JointConverter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 