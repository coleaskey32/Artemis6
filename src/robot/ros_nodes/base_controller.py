#!/usr/bin/env python3
"""
ROS2 Base Controller for Mobile Manipulator Robot
Subscribes to /cmd_vel and controls the 4-wheel mobile base via Webots socket
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import socket
import json
import time

class BaseController(Node):
    def __init__(self):
        super().__init__('base_controller')
        
        # Subscribe to cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Connect to Webots controller
        self.socket = None
        self.connect_to_webots()
        
        self.get_logger().info('🚗 Base Controller Started!')
        self.get_logger().info('📡 Listening to /cmd_vel topic...')
        self.get_logger().info('')
        self.get_logger().info('🎮 Test with:')
        self.get_logger().info('  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"')
    
    def connect_to_webots(self):
        """Connect to Webots controller socket"""
        max_retries = 5
        retry_delay = 2
        
        for attempt in range(max_retries):
            try:
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.connect(('localhost', 9999))
                self.get_logger().info('✅ Connected to Webots controller')
                return
            except ConnectionRefusedError:
                if attempt < max_retries - 1:
                    self.get_logger().warn(
                        f'⏳ Waiting for Webots controller... (attempt {attempt + 1}/{max_retries})'
                    )
                    time.sleep(retry_delay)
                else:
                    self.get_logger().error(
                        '❌ Failed to connect to Webots controller on localhost:9999'
                    )
                    self.get_logger().error('   Make sure Webots is running with the robot loaded!')
                    raise
    
    def cmd_vel_callback(self, msg):
        """Handle incoming velocity commands"""
        if self.socket is None:
            self.get_logger().warn('⚠️  Not connected to Webots')
            return
        
        # Create command message
        command = {
            'linear_x': msg.linear.x,
            'angular_z': msg.angular.z
        }
        
        try:
            # Send to Webots
            self.socket.sendall(json.dumps(command).encode('utf-8'))
            
            # Log the command
            if msg.linear.x != 0 or msg.angular.z != 0:
                self.get_logger().info(
                    f'🎮 Sent: linear={msg.linear.x:.2f} m/s, angular={msg.angular.z:.2f} rad/s'
                )
        except Exception as e:
            self.get_logger().error(f'❌ Failed to send command: {e}')
            # Try to reconnect
            try:
                self.connect_to_webots()
            except:
                pass
    
    def destroy_node(self):
        """Clean up on shutdown"""
        if self.socket:
            self.socket.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    controller = None
    
    try:
        controller = BaseController()
        rclpy.spin(controller)
    except KeyboardInterrupt:
        print('\n👋 Base Controller stopped')
    except Exception as e:
        print(f'❌ Error: {e}')
    finally:
        if controller is not None:
            controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

