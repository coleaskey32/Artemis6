#!/usr/bin/env python3
"""
ROS2 Arm Controller Node
Sends joint position commands to the Webots arm via socket
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import socket
import json
import time

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        self.get_logger().info("🦾 Arm Controller Started!")
        
        self.webots_host = 'localhost'
        self.webots_port = 9999
        self.socket = None
        self.connected = False
        
        # Subscribe to joint commands
        self.subscription = self.create_subscription(
            JointState,
            '/arm_joint_commands',
            self.joint_command_callback,
            10
        )
        
        # Connect to Webots controller
        self.connect_to_webots()
        
        self.get_logger().info("📡 Listening to /arm_joint_commands topic...")
        self.get_logger().info("")
        self.get_logger().info("🎮 Test arm motion:")
        self.get_logger().info("  ros2 topic pub /arm_joint_commands sensor_msgs/msg/JointState \"{position: [0.5, 0.3, -0.5, 0.0, 0.5, 0.0]}\"")
        self.get_logger().info("")
        self.get_logger().info("Joint Limits (radians):")
        self.get_logger().info("  Joint 1 (base):     -3.14 to  3.14")
        self.get_logger().info("  Joint 2 (shoulder): -1.57 to  1.57")
        self.get_logger().info("  Joint 3 (elbow):    -2.62 to  2.62")
        self.get_logger().info("  Joint 4 (wrist):    -3.14 to  3.14")
        self.get_logger().info("  Joint 5 (wrist):    -1.57 to  1.57")
        self.get_logger().info("  Joint 6 (wrist):    -3.14 to  3.14")
    
    def connect_to_webots(self):
        """Attempt to connect to the Webots controller socket."""
        reconnect_attempts = 5
        reconnect_delay = 2.0
        
        self.connected = False
        for attempt in range(reconnect_attempts):
            self.get_logger().warn(f"⏳ Connecting to Webots controller... (attempt {attempt + 1}/{reconnect_attempts})")
            try:
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.connect((self.webots_host, self.webots_port))
                self.socket.settimeout(1.0)
                self.connected = True
                self.get_logger().info("✅ Connected to Webots controller")
                return
            except (socket.error, ConnectionRefusedError) as e:
                self.get_logger().debug(f"Connection attempt failed: {e}")
                time.sleep(reconnect_delay)
        
        self.get_logger().error("❌ Failed to connect to Webots controller")
        self.get_logger().warn("⚠️  Make sure Webots is running with the robot simulation!")
    
    def joint_command_callback(self, msg):
        """Callback for incoming JointState messages."""
        if len(msg.position) != 6:
            self.get_logger().error(f"❌ Expected 6 joint positions, got {len(msg.position)}")
            return
        
        # Clamp joint positions to limits
        joint_limits = [
            (-3.14159, 3.14159),   # Joint 1: Base rotation
            (-1.5708, 1.5708),      # Joint 2: Shoulder
            (-2.618, 2.618),        # Joint 3: Elbow
            (-3.14159, 3.14159),   # Joint 4: Wrist roll
            (-1.5708, 1.5708),      # Joint 5: Wrist pitch
            (-3.14159, 3.14159),   # Joint 6: Wrist yaw
        ]
        
        clamped_positions = []
        for i, pos in enumerate(msg.position):
            min_limit, max_limit = joint_limits[i]
            clamped = max(min_limit, min(max_limit, pos))
            clamped_positions.append(clamped)
            
            if abs(clamped - pos) > 0.001:
                self.get_logger().warn(f"⚠️  Joint {i+1} clamped: {pos:.3f} → {clamped:.3f}")
        
        # Send command via socket
        command = {
            'arm_joints': clamped_positions
        }
        
        if self.connected:
            try:
                self.socket.sendall(json.dumps(command).encode('utf-8'))
                joint_str = ', '.join([f'{p:.3f}' for p in clamped_positions])
                self.get_logger().info(f"🦾 Sent: [{joint_str}]")
            except (socket.error, BrokenPipeError) as e:
                self.get_logger().error(f"❌ Failed to send command: {e}")
                self.connected = False
                self.socket.close()
                self.connect_to_webots()
        else:
            self.get_logger().warn("⚠️  Not connected to Webots controller")
    
    def run(self):
        """Main loop for the controller."""
        try:
            rclpy.spin(self)
        except KeyboardInterrupt:
            self.get_logger().info("👋 Shutting down Arm Controller.")
        finally:
            if self.socket:
                self.socket.close()
            self.destroy_node()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    controller = ArmController()
    controller.run()

if __name__ == '__main__':
    main()

