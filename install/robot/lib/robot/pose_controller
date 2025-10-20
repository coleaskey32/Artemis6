#!/usr/bin/env python3
"""
Pose Controller - Move arm to named poses
Publishes arm commands from a pose library
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from ament_index_python.packages import get_package_share_directory
import yaml
import os
import sys

class PoseController(Node):
    def __init__(self):
        super().__init__('pose_controller')
        
        # Publisher for arm commands
        self.publisher = self.create_publisher(
            JointState,
            '/arm_joint_commands',
            10
        )
        
        # Load poses
        self.poses = self.load_poses()
        
        self.get_logger().info("📚 Pose Controller Started!")
        self.get_logger().info(f"Loaded {len(self.poses)} poses")
        
    def load_poses(self):
        """Load poses from YAML file"""
        # Find the config file using ROS2 package share directory
        try:
            package_share_dir = get_package_share_directory('robot')
            config_path = os.path.join(package_share_dir, 'config', 'arm_poses.yaml')
            
            with open(config_path, 'r') as f:
                poses = yaml.safe_load(f)
                self.get_logger().info(f"✅ Loaded poses from {config_path}")
                return poses
        except FileNotFoundError:
            self.get_logger().error(f"❌ Pose file not found: {config_path}")
            return {}
        except Exception as e:
            self.get_logger().error(f"❌ Error loading poses: {e}")
            return {}
    
    def list_poses(self):
        """List all available poses"""
        self.get_logger().info("\n📚 Available Poses:")
        for name, data in self.poses.items():
            desc = data.get('description', 'No description')
            self.get_logger().info(f"  • {name}: {desc}")
    
    def move_to_pose(self, pose_name):
        """Move arm to a named pose"""
        if pose_name not in self.poses:
            self.get_logger().error(f"❌ Pose '{pose_name}' not found!")
            self.list_poses()
            return False
        
        pose = self.poses[pose_name]
        position = pose['position']
        description = pose.get('description', '')
        
        # Create and publish joint state message
        msg = JointState()
        msg.position = position
        
        self.publisher.publish(msg)
        
        self.get_logger().info(f"🦾 Moving to '{pose_name}': {description}")
        self.get_logger().info(f"   Joints: {[f'{p:.3f}' for p in position]}")
        
        return True

def main(args=None):
    rclpy.init(args=args)
    
    controller = PoseController()
    
    # Check command line arguments
    if len(sys.argv) < 2:
        controller.get_logger().info("\n🎮 Usage:")
        controller.get_logger().info("  ros2 run robot pose_controller <pose_name>")
        controller.get_logger().info("  ros2 run robot pose_controller list")
        controller.get_logger().info("")
        controller.list_poses()
        controller.destroy_node()
        rclpy.shutdown()
        return
    
    command = sys.argv[1]
    
    if command == "list":
        controller.list_poses()
    else:
        # Move to specified pose
        controller.move_to_pose(command)
        # Give it time to publish
        rclpy.spin_once(controller, timeout_sec=0.1)
    
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

