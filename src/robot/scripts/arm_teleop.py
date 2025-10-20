#!/usr/bin/env python3
"""
Arm Teleoperation - Control arm with keyboard
Arrow keys and WASD for joint control
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import sys
import termios
import tty
import select

class ArmTeleop(Node):
    def __init__(self):
        super().__init__('arm_teleop')
        
        # Publisher
        self.publisher = self.create_publisher(
            JointState,
            '/arm_joint_commands',
            10
        )
        
        # Current joint positions (start at home)
        self.joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Which joint is selected
        self.selected_joint = 0
        
        # Movement increment (radians)
        self.increment = 0.1  # ~5.7 degrees
        
        # Joint limits
        self.joint_limits = [
            (-3.14159, 3.14159),   # Joint 1
            (-1.5708, 1.5708),     # Joint 2
            (-2.618, 2.618),       # Joint 3
            (-3.14159, 3.14159),   # Joint 4
            (-1.5708, 1.5708),     # Joint 5
            (-3.14159, 3.14159),   # Joint 6
        ]
        
        # Joint names for display
        self.joint_names = [
            "Base (J1)",
            "Shoulder (J2)",
            "Elbow (J3)",
            "Wrist Roll (J4)",
            "Wrist Pitch (J5)",
            "Wrist Yaw (J6)"
        ]
        
        self.print_instructions()
        self.print_status()
        
    def print_instructions(self):
        """Print control instructions"""
        print("\n" + "="*60)
        print("🦾 ARM TELEOPERATION")
        print("="*60)
        print("\nControls:")
        print("  1-6     : Select joint (1=Base, 2=Shoulder, etc.)")
        print("  ↑/w     : Increase selected joint angle")
        print("  ↓/s     : Decrease selected joint angle")
        print("  +/-     : Change increment size")
        print("  h       : Go to home position")
        print("  0       : Zero selected joint")
        print("  SPACE   : Print current status")
        print("  q/ESC   : Quit")
        print("="*60 + "\n")
        
    def print_status(self):
        """Print current arm status"""
        print(f"\n{'='*60}")
        print(f"Selected: {self.joint_names[self.selected_joint]} ← ACTIVE")
        print(f"Increment: {self.increment:.3f} rad ({self.increment * 57.3:.1f}°)")
        print(f"{'='*60}")
        
        for i, (name, pos) in enumerate(zip(self.joint_names, self.joint_positions)):
            min_lim, max_lim = self.joint_limits[i]
            deg = pos * 57.2958  # Convert to degrees
            marker = " ←" if i == self.selected_joint else ""
            
            # Progress bar
            range_size = max_lim - min_lim
            normalized = (pos - min_lim) / range_size
            bar_length = 20
            filled = int(normalized * bar_length)
            bar = "█" * filled + "░" * (bar_length - filled)
            
            print(f"  {name:20} [{bar}] {pos:6.3f} rad ({deg:6.1f}°){marker}")
        
        print("="*60 + "\n")
    
    def get_key(self):
        """Get a single keypress"""
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1)
        return None
    
    def move_joint(self, delta):
        """Move selected joint by delta"""
        new_pos = self.joint_positions[self.selected_joint] + delta
        
        # Clamp to limits
        min_lim, max_lim = self.joint_limits[self.selected_joint]
        new_pos = max(min_lim, min(max_lim, new_pos))
        
        self.joint_positions[self.selected_joint] = new_pos
        self.publish_command()
        
        # Just print the updated joint (with \r\n for raw terminal mode)
        deg = new_pos * 57.2958
        name = self.joint_names[self.selected_joint]
        print(f"\r  {name:20} {new_pos:6.3f} rad ({deg:6.1f}°)", end='\r\n', flush=True)
    
    def select_joint(self, joint_index):
        """Select a joint"""
        if 0 <= joint_index < 6:
            self.selected_joint = joint_index
            deg = self.joint_positions[joint_index] * 57.2958
            name = self.joint_names[joint_index]
            print(f"\r✓ Selected: {name} (currently {self.joint_positions[joint_index]:.3f} rad / {deg:.1f}°)", end='\r\n', flush=True)
    
    def go_home(self):
        """Return to home position"""
        self.joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.publish_command()
        print("\r🏠 Home position (all joints → 0.0 rad)", end='\r\n', flush=True)
    
    def zero_joint(self):
        """Zero the selected joint"""
        self.joint_positions[self.selected_joint] = 0.0
        self.publish_command()
        name = self.joint_names[self.selected_joint]
        print(f"\r  {name:20} → 0.0 rad (zeroed)", end='\r\n', flush=True)
    
    def publish_command(self):
        """Publish current joint positions"""
        msg = JointState()
        msg.position = self.joint_positions
        self.publisher.publish(msg)
    
    def run(self):
        """Main control loop"""
        # Set up terminal for raw input
        settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        
        try:
            while rclpy.ok():
                key = self.get_key()
                
                if key:
                    # Quit
                    if key in ['q', 'Q', '\x1b']:  # q, Q, or ESC
                        print("\r\n👋 Goodbye!", end='\r\n', flush=True)
                        break
                    
                    # Select joint (1-6)
                    elif key in '123456':
                        self.select_joint(int(key) - 1)
                    
                    # Move up
                    elif key in ['w', 'W'] or key == '\x1b[A':  # w or up arrow
                        self.move_joint(self.increment)
                    
                    # Move down
                    elif key in ['s', 'S'] or key == '\x1b[B':  # s or down arrow
                        self.move_joint(-self.increment)
                    
                    # Increase increment
                    elif key in ['+', '=']:
                        self.increment = min(0.5, self.increment + 0.05)
                        print(f"\r  Step size: {self.increment:.3f} rad ({self.increment * 57.3:.1f}°)", end='\r\n', flush=True)
                    
                    # Decrease increment
                    elif key in ['-', '_']:
                        self.increment = max(0.01, self.increment - 0.05)
                        print(f"\r  Step size: {self.increment:.3f} rad ({self.increment * 57.3:.1f}°)", end='\r\n', flush=True)
                    
                    # Home
                    elif key in ['h', 'H']:
                        self.go_home()
                    
                    # Zero selected joint
                    elif key == '0':
                        self.zero_joint()
                    
                    # Print status
                    elif key == ' ':
                        self.print_status()
                    
                    # Help
                    elif key == '?':
                        self.print_instructions()
                        self.print_status()
                
                # Spin to process callbacks
                rclpy.spin_once(self, timeout_sec=0.01)
                
        finally:
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

def main(args=None):
    rclpy.init(args=args)
    teleop = ArmTeleop()
    
    try:
        teleop.run()
    except KeyboardInterrupt:
        pass
    finally:
        teleop.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

