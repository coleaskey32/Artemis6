#!/usr/bin/env python3
"""
Pick and Place Demo
Demonstrates IK by picking up the soup can and moving it
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import time

class PickAndPlaceDemo(Node):
    def __init__(self):
        super().__init__('pick_and_place_demo')
        
        # Publisher for target poses
        self.pose_publisher = self.create_publisher(
            PoseStamped,
            '/gripper_target_pose',
            10
        )
        
        self.get_logger().info("🤖 Pick and Place Demo Started!")
        self.get_logger().info("Will pick up the red soup can in 3 seconds...")
        
    def publish_target(self, x, y, z, description="Moving"):
        """Publish a target gripper pose"""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0  # No rotation
        
        self.pose_publisher.publish(msg)
        self.get_logger().info(f"📍 {description}: ({x:.3f}, {y:.3f}, {z:.3f})")
    
    def run_demo(self):
        """Run the pick and place sequence"""
        
        # Wait for everything to be ready
        time.sleep(3)
        
        # Sequence of movements
        self.get_logger().info("\n🎬 Starting Pick and Place Sequence!")
        self.get_logger().info("=" * 60)
        
        # Step 1: Move to home position
        self.get_logger().info("\n1️⃣  Moving to home position above robot")
        self.publish_target(0.2, 0.0, 0.3, "Home position")
        time.sleep(3)
        
        # Step 2: Move above the can
        self.get_logger().info("\n2️⃣  Moving above the soup can")
        self.publish_target(0.4, 0.0, 0.25, "Above can")
        time.sleep(3)
        
        # Step 3: Move down to grasp
        self.get_logger().info("\n3️⃣  Lowering to grasp the can")
        self.publish_target(0.4, 0.0, 0.13, "Grasp height")
        time.sleep(3)
        
        self.get_logger().info("🤏 [GRASP] Close gripper here (not implemented yet)")
        time.sleep(2)
        
        # Step 4: Lift the can
        self.get_logger().info("\n4️⃣  Lifting the can")
        self.publish_target(0.4, 0.0, 0.3, "Lift up")
        time.sleep(3)
        
        # Step 5: Move to new location
        self.get_logger().info("\n5️⃣  Moving to place location")
        self.publish_target(0.3, 0.2, 0.3, "Above drop zone")
        time.sleep(3)
        
        # Step 6: Lower to place
        self.get_logger().info("\n6️⃣  Lowering to place")
        self.publish_target(0.3, 0.2, 0.13, "Place height")
        time.sleep(3)
        
        self.get_logger().info("✋ [RELEASE] Open gripper here (not implemented yet)")
        time.sleep(2)
        
        # Step 7: Move up and away
        self.get_logger().info("\n7️⃣  Retracting")
        self.publish_target(0.3, 0.2, 0.3, "Retract")
        time.sleep(3)
        
        # Step 8: Return home
        self.get_logger().info("\n8️⃣  Returning home")
        self.publish_target(0.2, 0.0, 0.3, "Home")
        time.sleep(3)
        
        self.get_logger().info("\n" + "=" * 60)
        self.get_logger().info("✅ Demo Complete!")
        self.get_logger().info("\n💡 Note: Gripper open/close not implemented yet")
        self.get_logger().info("   The arm moves through the full sequence!")

def main(args=None):
    rclpy.init(args=args)
    
    demo = PickAndPlaceDemo()
    
    try:
        demo.run_demo()
        rclpy.spin(demo)
    except KeyboardInterrupt:
        demo.get_logger().info("\n👋 Demo stopped")
    finally:
        demo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

