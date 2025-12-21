#!/usr/bin/env python3
"""
ROS2 Arm Controller Node
Sends joint position commands to the Webots arm via socket
5-DOF Arm + Prismatic Gripper
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
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
        
        # Subscribe to joint commands (5 DOF arm)
        self.subscription = self.create_subscription(
            JointState,
            '/arm_joint_commands',
            self.joint_command_callback,
            10
        )
        
        # Subscribe to gripper commands (separate topic)
        self.gripper_subscription = self.create_subscription(
           Float64,
           '/gripper_command',
           self.gripper_command_callback,
           10
        )
        
        # Connect to Webots controller
        self.connect_to_webots()
        
        self.get_logger().info("📡 Listening to /arm_joint_commands and /gripper_command topics...")
        self.get_logger().info("")
        self.get_logger().info("🎮 Test arm motion:")
        self.get_logger().info("  ros2 topic pub /arm_joint_commands sensor_msgs/msg/JointState \"{position: [0.5, 0.5, 0.5, 0.5, 0.0]}\"")
        self.get_logger().info("")
        self.get_logger().info("🤏 Test gripper:")
        self.get_logger().info("  ros2 topic pub /gripper_command std_msgs/msg/Float64 \"{data: 0.03}\"  # Open")
        self.get_logger().info("  ros2 topic pub /gripper_command std_msgs/msg/Float64 \"{data: 0.0}\"   # Close")
        self.get_logger().info("")
        self.get_logger().info("Joint Limits:")
        self.get_logger().info("  Joint 1 (base_yaw):       -3.14 to  3.14 rad")
        self.get_logger().info("  Joint 2 (shoulder):     -1.5708 to  1.5708 rad")
        self.get_logger().info("  Joint 3 (elbow):           0.00 to  1.57 rad")
        self.get_logger().info("  Joint 4 (wrist_pitch):     0.00 to  1.57 rad")
        self.get_logger().info("  Joint 5 (wrist_roll):     -3.14 to  3.14 rad")
        self.get_logger().info("  Gripper (prismatic):       0.00 to  0.03 m")
    
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
    
    def gripper_command_callback(self, msg):
        """Callback for incoming gripper commands (Float64: 0.0 to 0.03 meters)."""
        # Clamp to gripper limits
        gripper_pos = max(0.0, min(0.03, msg.data))
        
        if abs(gripper_pos - msg.data) > 0.0001:
            self.get_logger().warn(f"⚠️  Gripper clamped: {msg.data:.4f} → {gripper_pos:.4f}")
        
        command = {'gripper': gripper_pos}
        
        if self.connected:
            try:
                self.socket.sendall(json.dumps(command).encode('utf-8'))
                self.get_logger().info(f"🤏 Gripper: {gripper_pos:.4f} m")
            except (socket.error, BrokenPipeError) as e:
                self.get_logger().error(f"❌ Failed to send gripper command: {e}")
                self.connected = False
                self.socket.close()
                self.connect_to_webots()
        else:
            self.get_logger().warn("⚠️  Not connected to Webots controller")
    
    def joint_command_callback(self, msg):
        """Callback for incoming JointState messages (5 DOF arm)."""
        if len(msg.position) != 5:
            self.get_logger().error(f"❌ Expected 5 joint positions, got {len(msg.position)}")
            return
        
        # Clamp joint positions to limits (matching new_robot.urdf)
        joint_limits = [
            (-3.14159, 3.14159),   # Joint 1: base_yaw_joint (Z rotation)
            (-1.5708, 1.5708),          # Joint 2: shoulder_joint (Y pitch)
            (0.0, 1.5708),          # Joint 3: elbow_joint (Y pitch)
            (-1.5708, 1.5708),      # Joint 4: wrist_pitch_joint (Y pitch)
            (-3.14159, 3.14159),   # Joint 5: wrist_roll_joint (Z rotation)
        ]
        
        joint_names = [
            'base_yaw',
            'shoulder',
            'elbow',
            'wrist_pitch',
            'wrist_roll'
        ]
        
        clamped_positions = []
        for i, pos in enumerate(msg.position):
            min_limit, max_limit = joint_limits[i]
            clamped = max(min_limit, min(max_limit, pos))
            clamped_positions.append(clamped)
            
            if abs(clamped - pos) > 0.001:
                self.get_logger().warn(
                    f"⚠️  {joint_names[i]} clamped: {pos:.3f} → {clamped:.3f}"
                )
        
        # Send command via socket
        command = {
            'arm_joints': clamped_positions
        }
        
        if self.connected:
            try:
                self.socket.sendall(json.dumps(command).encode('utf-8'))
                joint_str = ', '.join([f'{p:.3f}' for p in clamped_positions])
                self.get_logger().info(f"🦾 Arm: [{joint_str}]")
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

