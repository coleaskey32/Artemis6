#!/usr/bin/env python3
"""
ROS2 Keyboard Teleop for Mobile Manipulator
Control base movement and arm joints with keyboard
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import sys
import termios
import tty
import select

HELP_MSG = """
╔═══════════════════════════════════════════════════════════════════╗
║              🎮 Mobile Manipulator Keyboard Control 🎮            ║
╠═══════════════════════════════════════════════════════════════════╣
║  BASE MOVEMENT:                                                   ║
║    W/S : Forward / Backward                                       ║
║    A/D : Turn Left / Turn Right                                   ║
║    X   : Stop base                                                ║
║                                                                   ║
║  ARM CONTROL:                                                     ║
║    1/2 : Joint 1 (Base Yaw)     ↻/↺                               ║
║    3/4 : Joint 2 (Shoulder)     Up/Down                           ║
║    5/6 : Joint 3 (Elbow)        Bend/Extend                       ║
║    7/8 : Joint 4 (Wrist Pitch)  Up/Down                           ║
║    9/0 : Joint 5 (Wrist Roll)   ↻/↺                               ║
║                                                                   ║
║  GRIPPER:                                                         ║
║    O   : Open gripper                                             ║
║    C   : Close gripper                                            ║
║                                                                   ║
║  SPEED:                                                           ║
║    +/= : Increase speed                                           ║
║    -   : Decrease speed                                           ║
║                                                                   ║
║  H : Show this help     Q/Ctrl+C : Quit                           ║
╚═══════════════════════════════════════════════════════════════════╝
"""

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_control')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_pub = self.create_publisher(JointState, '/arm_joint_commands', 10)
        self.gripper_pub = self.create_publisher(Float64, '/gripper_command', 10)
        
        # Speed settings
        self.linear_speed = 0.5  # m/s
        self.angular_speed = 1.0  # rad/s
        self.joint_step = 0.1    # radians per keypress
        
        # Current arm joint positions (5 DOF)
        self.arm_positions = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Joint limits (from arm_controller.py)
        self.joint_limits = [
            (-3.14159, 3.14159),   # Joint 1: base_yaw
            (-1.5708, 1.5708),     # Joint 2: shoulder
            (0.0, 1.5708),         # Joint 3: elbow
            (-1.5708, 1.5708),     # Joint 4: wrist_pitch
            (-3.14159, 3.14159),   # Joint 5: wrist_roll
        ]
        
        # Gripper state
        self.gripper_position = 0.0  # 0.0 = closed, 0.03 = open
        
        # Save terminal settings
        self.old_settings = termios.tcgetattr(sys.stdin)
        
        self.get_logger().info('🎮 Keyboard Teleop Started!')
        print(HELP_MSG)
    
    def get_key(self):
        """Get a single keypress without blocking."""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        return key
    
    def publish_cmd_vel(self, linear_x, angular_z):
        """Publish velocity command."""
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_vel_pub.publish(msg)
        if linear_x != 0 or angular_z != 0:
            self.get_logger().info(f'🚗 Base: linear={linear_x:.2f}, angular={angular_z:.2f}')
    
    def publish_arm_position(self):
        """Publish current arm joint positions."""
        msg = JointState()
        msg.position = self.arm_positions
        self.arm_pub.publish(msg)
        joints_str = ', '.join([f'{p:.2f}' for p in self.arm_positions])
        self.get_logger().info(f'🦾 Arm: [{joints_str}]')
    
    def publish_gripper(self):
        """Publish gripper position."""
        msg = Float64()
        msg.data = self.gripper_position
        self.gripper_pub.publish(msg)
        state = "OPEN" if self.gripper_position > 0.01 else "CLOSED"
        self.get_logger().info(f'🤏 Gripper: {state} ({self.gripper_position:.3f}m)')
    
    def adjust_joint(self, joint_idx, direction):
        """Adjust a joint position within limits."""
        new_pos = self.arm_positions[joint_idx] + (direction * self.joint_step)
        min_limit, max_limit = self.joint_limits[joint_idx]
        self.arm_positions[joint_idx] = max(min_limit, min(max_limit, new_pos))
        self.publish_arm_position()
    
    def run(self):
        """Main loop."""
        try:
            while rclpy.ok():
                key = self.get_key()
                
                if not key:
                    continue
                
                key = key.lower()
                
                # Quit
                if key == 'q' or key == '\x03':  # q or Ctrl+C
                    self.get_logger().info('👋 Shutting down...')
                    break
                
                # Help
                elif key == 'h':
                    print(HELP_MSG)
                
                # Base movement
                elif key == 'w':
                    self.publish_cmd_vel(self.linear_speed, 0.0)
                elif key == 's':
                    self.publish_cmd_vel(-self.linear_speed, 0.0)
                elif key == 'a':
                    self.publish_cmd_vel(0.0, self.angular_speed)
                elif key == 'd':
                    self.publish_cmd_vel(0.0, -self.angular_speed)
                elif key == 'x':
                    self.publish_cmd_vel(0.0, 0.0)
                    self.get_logger().info('🛑 Base stopped')
                
                # Arm joints
                elif key == '1':
                    self.adjust_joint(0, 1)   # Joint 1 +
                elif key == '2':
                    self.adjust_joint(0, -1)  # Joint 1 -
                elif key == '3':
                    self.adjust_joint(1, 1)   # Joint 2 +
                elif key == '4':
                    self.adjust_joint(1, -1)  # Joint 2 -
                elif key == '5':
                    self.adjust_joint(2, 1)   # Joint 3 +
                elif key == '6':
                    self.adjust_joint(2, -1)  # Joint 3 -
                elif key == '7':
                    self.adjust_joint(3, 1)   # Joint 4 +
                elif key == '8':
                    self.adjust_joint(3, -1)  # Joint 4 -
                elif key == '9':
                    self.adjust_joint(4, 1)   # Joint 5 +
                elif key == '0':
                    self.adjust_joint(4, -1)  # Joint 5 -
                
                # Gripper
                elif key == 'o':
                    self.gripper_position = 0.03  # Open
                    self.publish_gripper()
                elif key == 'c':
                    self.gripper_position = 0.0   # Close
                    self.publish_gripper()
                
                # Speed adjustment
                elif key == '+' or key == '=':
                    self.linear_speed = min(2.0, self.linear_speed + 0.1)
                    self.angular_speed = min(3.0, self.angular_speed + 0.2)
                    self.get_logger().info(f'⚡ Speed: linear={self.linear_speed:.1f}, angular={self.angular_speed:.1f}')
                elif key == '-':
                    self.linear_speed = max(0.1, self.linear_speed - 0.1)
                    self.angular_speed = max(0.2, self.angular_speed - 0.2)
                    self.get_logger().info(f'⚡ Speed: linear={self.linear_speed:.1f}, angular={self.angular_speed:.1f}')
                
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
        finally:
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            # Stop the robot
            self.publish_cmd_vel(0.0, 0.0)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
