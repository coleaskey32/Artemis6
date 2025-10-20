#!/usr/bin/env python3
"""
Square Drive Node
Makes the robot drive in a square pattern by publishing to /cmd_vel
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class SquareDrive(Node):
    def __init__(self):
        super().__init__('square_drive')
        
        # Publisher to /cmd_vel
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Robot parameters
        self.linear_speed = 0.5  # m/s
        self.angular_speed = 0.5  # rad/s
        self.side_length = 2.0  # meters
        
        # Calculate durations
        self.forward_duration = self.side_length / self.linear_speed  # seconds
        self.turn_duration = (3.14159 / 2) / self.angular_speed  # 90 degrees in radians / angular speed
        
        self.get_logger().info('🟦 Square Drive Node Started!')
        self.get_logger().info(f'📏 Side length: {self.side_length}m')
        self.get_logger().info(f'⏱️  Forward time: {self.forward_duration}s')
        self.get_logger().info(f'🔄 Turn time: {self.turn_duration}s')
        self.get_logger().info('')
        self.get_logger().info('Press Ctrl+C to stop')
        self.get_logger().info('')
    
    def stop(self):
        """Send stop command"""
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher.publish(msg)
        self.get_logger().info('⏸️  Stop')
    
    def move_forward(self, duration):
        """Move forward for specified duration"""
        self.get_logger().info(f'➡️  Moving forward for {duration:.1f}s...')
        msg = Twist()
        msg.linear.x = self.linear_speed
        msg.angular.z = 0.0
        
        start_time = time.time()
        while (time.time() - start_time) < duration:
            self.publisher.publish(msg)
            time.sleep(0.1)
        
        self.stop()
    
    def turn_90_degrees(self):
        """Turn 90 degrees (counter-clockwise)"""
        self.get_logger().info(f'🔄 Turning 90° for {self.turn_duration:.1f}s...')
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = self.angular_speed
        
        start_time = time.time()
        while (time.time() - start_time) < self.turn_duration:
            self.publisher.publish(msg)
            time.sleep(0.1)
        
        self.stop()
    
    def drive_square(self, num_squares=1):
        """Drive in a square pattern"""
        self.get_logger().info(f'🟦 Starting square drive ({num_squares} square(s))...')
        self.get_logger().info('')
        
        for square_num in range(num_squares):
            self.get_logger().info(f'--- Square {square_num + 1} of {num_squares} ---')
            
            for side in range(4):
                self.get_logger().info(f'Side {side + 1}/4:')
                
                # Move forward
                self.move_forward(self.forward_duration)
                time.sleep(0.5)  # Pause between movements
                
                # Turn 90 degrees (unless it's the last side)
                if side < 3 or square_num < num_squares - 1:
                    self.turn_90_degrees()
                    time.sleep(0.5)  # Pause between movements
            
            self.get_logger().info(f'✅ Square {square_num + 1} completed!')
            self.get_logger().info('')
        
        self.get_logger().info('🎉 All squares completed!')
        self.stop()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = SquareDrive()
        
        # Drive in 1 square by default (change this number for multiple squares)
        node.drive_square(num_squares=1)
        
        # Keep node alive briefly to ensure final stop command is sent
        time.sleep(1)
        
    except KeyboardInterrupt:
        print('\n👋 Square drive stopped by user')
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()

