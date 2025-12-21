#!/usr/bin/env python3
"""
Pick and Place Demo
Demonstrates picking up an object and placing it somewhere else
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float64, Empty
import time

class PickPlaceDemo(Node):
    def __init__(self):
        super().__init__('pick_place_demo')
        
        # Publishers
        self.arm_pub = self.create_publisher(Point, '/arm_coordinates', 10)
        self.gripper_pub = self.create_publisher(Float64, '/gripper_command', 10)
        self.home_pub = self.create_publisher(Empty, '/arm_home', 10)
        
        # Timing (seconds to wait for each motion)
        self.arm_move_time = 2.5   # Time for arm to reach position
        self.gripper_time = 0.5    # Time for gripper to open/close
        self.home_time = 2.0       # Time to reach home (includes IK delay)
        
        # Object positions in the world (from mobile_manipulator.wbt)
        self.objects = {
            'red_box' : (0.5, 0.0, 0.0),
            'green_cylinder' : (-0.5, 0.3, 0.0),
            'blue_sphere' : (0.3, -0.4, 0.0)
        }
        
        self.get_logger().info("🤖 Pick & Place Demo Ready!")
        self.get_logger().info("")
        self.get_logger().info("Commands:")
        self.get_logger().info("   pick <object>  - Pick up an object")
        self.get_logger().info("   place          - Place held object")
        self.get_logger().info("   demo           - Full pick & place demo")
        self.get_logger().info("   home           - Return to home")
        self.get_logger().info("   quit           - Exit")
    
    def wait(self, seconds):
        """Wait while processing ROS callbacks."""
        time.sleep(seconds)
    
    def go_home(self):
        """Send arm to home position."""
        self.get_logger().info("🏠 Going home...")
        self.home_pub.publish(Empty())
        self.wait(self.home_time)
    
    def move_arm(self, x, y, z):
        """Move arm to XYZ position."""
        self.get_logger().info(f"🦾 Moving to ({x:.2f}, {y:.2f}, {z:.2f})...")
        msg = Point()
        msg.x, msg.y, msg.z = x, y, z
        self.arm_pub.publish(msg)
        self.wait(self.arm_move_time)
    
    def open_gripper(self):
        """Open the gripper."""
        self.get_logger().info("🤏 Opening gripper...")
        msg = Float64()
        msg.data = 0.03  # Fully open
        self.gripper_pub.publish(msg)
        self.wait(self.gripper_time)
    
    def close_gripper(self):
        """Close the gripper."""
        self.get_logger().info("🤏 Closing gripper...")
        msg = Float64()
        msg.data = 0.0  # Fully closed
        self.gripper_pub.publish(msg)
        self.wait(self.gripper_time)
    
    def pick_object(self, object_name):
        """Pick up a specified object."""
        if object_name not in self.objects:
            self.get_logger().error(f"❌ Unknown object: {object_name}")
            return False
        
        x, y, z = self.objects[object_name]

        # Step 1: Move above the object
        self.get_logger().info("⬆️ Moving above the object...")
        self.move_arm(x, y, z + 0.12)
        
        self.wait(2)
        
        # Step 2: Open gripper
        self.open_gripper()
        
        self.wait(2)
        
        # Step 3: Lower to grab height
        self.get_logger().info("⬇️ Lowering to grab...")
        self.move_arm(x, y, z)
        
        self.wait(2)
        
        # Step 4: Close gripper
        self.close_gripper()
        
        self.wait(2)
        
        # Step 5: Lift object
        self.get_logger().info("⬆️ Lifting object...")
        self.move_arm(x, y, z + 0.3)
        
        self.wait(2)
        
        self.get_logger().info(f"✅ Picked up {object_name}!")
        return True
    
    def place_object(self, x, y, z):
        """Place object at specified position."""
        approach_z = z + 0.15
        
        self.get_logger().info(f"📍 Placing at ({x:.2f}, {y:.2f}, {z:.2f})...")
        
        # Step 1: Move above place position
        self.get_logger().info("➡️ Moving to place position...")
        self.move_arm(x, y, approach_z)
        
        # Step 2: Lower to place height
        self.get_logger().info("⬇️ Lowering...")
        self.move_arm(x, y, z)
        
        # Step 3: Open gripper
        self.open_gripper()
        
        # Step 4: Lift away
        self.get_logger().info("⬆️ Lifting away...")
        self.move_arm(x, y, approach_z)
        
        self.get_logger().info("✅ Object placed!")
    
    def run_demo(self):
        """Run a full pick and place demonstration."""
        self.get_logger().info("🎬 Starting Pick & Place Demo!")
        self.get_logger().info("=" * 50)
        
        # Pick up the red box
        if self.pick_object('red_box'):
            # Place it at a new location
            place_pos = self.place_positions[0]
            self.place_object(*place_pos)
        
        # Return home
        self.go_home()
        
        self.get_logger().info("=" * 50)
        self.get_logger().info("🎬 Demo Complete!")
    
    def run_interactive(self):
        """Run interactive command mode."""
        self.get_logger().info("\n🎮 Interactive Mode - Type commands below:")
        
        while rclpy.ok():
            try:
                cmd = input("\n> ").strip().lower()
                
                if cmd == 'quit' or cmd == 'q':
                    break
                elif cmd == 'home':
                    self.go_home()
                elif cmd == 'demo':
                    self.run_demo()
                elif cmd == 'open':
                    self.open_gripper()
                elif cmd == 'close':
                    self.close_gripper()
                elif cmd.startswith('pick '):
                    obj_name = cmd.split(' ', 1)[1]
                    self.pick_object(obj_name)
                elif cmd == 'place':
                    self.place_object(*self.place_positions[0])
                elif cmd.startswith('move '):
                    # Parse: move x y z
                    parts = cmd.split()
                    if len(parts) == 4:
                        x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                        self.move_arm(x, y, z)
                    else:
                        self.get_logger().info("Usage: move x y z")
                elif cmd == 'help':
                    self.get_logger().info("Commands: pick <obj>, place, move x y z, open, close, home, demo, quit")
                else:
                    self.get_logger().info("Unknown command. Type 'help' for options.")
                    
            except KeyboardInterrupt:
                break
            except Exception as e:
                self.get_logger().error(f"Error: {e}")
        
        self.get_logger().info("👋 Goodbye!")

def main(args=None):
    rclpy.init(args=args)
    demo = PickPlaceDemo()
    
    try:
        demo.run_interactive()
    finally:
        demo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

