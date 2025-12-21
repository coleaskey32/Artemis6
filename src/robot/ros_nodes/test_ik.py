#!/usr/bin/env python3
"""
IK Accuracy Test Script
Tests the IK solver across multiple points in the workspace
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import time

class IKTester(Node):
    def __init__(self):
        super().__init__('ik_tester')
        
        self.publisher = self.create_publisher(Point, '/arm_coordinates', 10)
        
        # Test points within the robot's workspace
        # Format: (x, y, z) in meters
        self.test_points = [
            # Front positions
            (0.3, 0.0, 0.4),   # Front center, medium height
            (0.4, 0.0, 0.3),   # Front center, lower
            (0.3, 0.0, 0.6),   # Front center, higher
            
            # Side positions
            (0.2, 0.3, 0.4),   # Front-left
            (0.2, -0.3, 0.4),  # Front-right
            
            # Diagonal positions
            (0.3, 0.2, 0.3),   # Diagonal left
            (0.3, -0.2, 0.3),  # Diagonal right
            
            # Edge of workspace (may have higher error)
            (0.5, 0.0, 0.2),   # Far front, low
            (0.0, 0.4, 0.5),   # Side reach
        ]
        
        self.current_test = 0
        self.get_logger().info("🧪 IK Tester Started!")
        self.get_logger().info(f"   Testing {len(self.test_points)} positions")
        self.get_logger().info("   Watch the IK Solver output for errors!")
        self.get_logger().info("")
        self.get_logger().info("Press Enter to test each point...")
        
    def run_tests(self):
        """Run through all test points interactively."""
        for i, point in enumerate(self.test_points):
            input(f"\n[{i+1}/{len(self.test_points)}] Press Enter to test {point}...")
            
            msg = Point()
            msg.x, msg.y, msg.z = point
            self.publisher.publish(msg)
            
            self.get_logger().info(f"📤 Sent: x={point[0]}, y={point[1]}, z={point[2]}")
            
            # Wait for arm to move
            time.sleep(3.0)
        
        self.get_logger().info("\n✅ All tests complete!")
        self.get_logger().info("Check the IK Solver output for error values.")

def main(args=None):
    rclpy.init(args=args)
    tester = IKTester()
    
    try:
        tester.run_tests()
    except KeyboardInterrupt:
        tester.get_logger().info("👋 Test cancelled")
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

