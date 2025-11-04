#!/usr/bin/env python3
"""
Joint State Publisher
Reads arm sensor data from Webots and publishes to ROS2
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import socket
import json
import time

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        
        # Publisher for joint states
        self.publisher = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )
        
        # Socket connection to Webots
        self.webots_host = 'localhost'
        self.webots_port = 9998  # Different port from commands
        self.socket = None
        self.connected = False
        
        # Joint names
        self.joint_names = [
            'arm_joint_1',
            'arm_joint_2',
            'arm_joint_3',
            'arm_joint_4',
            'arm_joint_5',
            'arm_joint_6'
        ]
        
        # Timer to publish joint states
        self.timer = self.create_timer(0.1, self.publish_joint_states)  # 10 Hz
        
        self.get_logger().info("📡 Joint State Publisher Started!")
        self.get_logger().info("Publishing joint states to /joint_states at 10 Hz")
        
    def publish_joint_states(self):
        """Publish current joint positions"""
        # For now, we'll use a simple approach
        # In a full implementation, this would read from Webots sensors
        
        # Create message
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        
        # TODO: Get actual positions from Webots
        # For now, publish zeros (will update in robot_controller.py)
        msg.position = [0.0] * 6
        msg.velocity = []
        msg.effort = []
        
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    publisher = JointStatePublisher()
    
    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        publisher.get_logger().info("👋 Shutting down Joint State Publisher")
    finally:
        publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

