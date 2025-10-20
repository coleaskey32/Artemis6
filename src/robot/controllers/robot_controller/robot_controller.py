#!/usr/bin/env python3
"""
Mobile Manipulator Webots Controller with Socket Bridge
Controls 4-wheel base + 6-axis robotic arm
Receives commands via socket from ROS2 bridge
"""

from controller import Robot
import socket
import json
import threading

class MobileManipulatorController:
    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        
        # Set up robot components
        self.setup_base()
        self.setup_arm()
        
        # Start socket server
        self.setup_socket_server()
        
        # Print startup messages
        print("🤖 Mobile Manipulator Controller Started!")
        print("   ├─ 🚗 4-Wheel Mobile Base: Ready")
        print("   └─ 🦾 Professional 6-Axis Robotic Arm: Ready")
        print()
        print("📡 Listening for ROS2 commands on port 9999")
        print("   ├─ Base control: /cmd_vel  (via base_controller)")
        print("   └─ Arm control: /arm_joint_commands  (via arm_controller)")
        print()
        print("Start controllers:")
        print("  ros2 run robot base_controller")
        print("  ros2 run robot arm_controller")
        
    def get_logger(self):
        """Simple logger that prints to stdout"""
        class SimpleLogger:
            def error(self, msg): print(f"\033[91m{msg}\033[0m")  # Red
            def warn(self, msg): print(f"\033[93m{msg}\033[0m")   # Yellow
            def info(self, msg): print(msg)                       # Normal
            def debug(self, msg): print(f"\033[90m{msg}\033[0m")  # Gray
        return SimpleLogger()
    
    def setup_base(self):
        """Set up the mobile base motors and parameters"""
        # Get wheel motor devices
        self.front_left = self.robot.getDevice('front_left_wheel')
        self.front_right = self.robot.getDevice('front_right_wheel')
        self.rear_left = self.robot.getDevice('rear_left_wheel')
        self.rear_right = self.robot.getDevice('rear_right_wheel')
        
        # Check if all motors were found
        for name, motor in [
            ('front_left_wheel', self.front_left),
            ('front_right_wheel', self.front_right),
            ('rear_left_wheel', self.rear_left),
            ('rear_right_wheel', self.rear_right)
        ]:
            if motor is None:
                self.get_logger().error(f"❌ Motor '{name}' not found!")
                raise RuntimeError(f"Motor '{name}' not found")
        
        # Set wheel motors to velocity control mode
        for motor in [self.front_left, self.front_right, self.rear_left, self.rear_right]:
            motor.setPosition(float('inf'))
            motor.setVelocity(0.0)
        
        # Mobile base parameters
        self.wheel_radius = 0.08  # meters
        self.wheel_separation = 0.4  # meters
        
        # Current base velocities
        self.linear_x = 0.0
        self.angular_z = 0.0
        
    def setup_arm(self):
        """Set up the robotic arm motors and sensors"""
        # Get arm joint motors (6-axis)
        self.arm_motors = []
        self.arm_sensors = []
        for i in range(1, 7):
            motor = self.robot.getDevice(f'arm_joint_{i}')
            sensor = self.robot.getDevice(f'arm_joint_{i}_sensor')
            
            if motor is None:
                self.get_logger().error(f"❌ Motor 'arm_joint_{i}' not found!")
                raise RuntimeError(f"Motor 'arm_joint_{i}' not found")
            
            if sensor is None:
                self.get_logger().error(f"❌ Sensor 'arm_joint_{i}_sensor' not found!")
                raise RuntimeError(f"Sensor 'arm_joint_{i}_sensor' not found")
            
            # Enable position sensors
            sensor.enable(self.timestep)
            
            # Set motors to position control mode (default)
            # Initialize to home position (all zeros)
            motor.setPosition(0.0)
            
            self.arm_motors.append(motor)
            self.arm_sensors.append(sensor)
        
        # Target arm joint positions (radians)
        self.arm_target_positions = [0.0] * 6
    
    def setup_socket_server(self):
        """Set up socket server in a separate thread"""
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind(('localhost', 9999))
        self.server_socket.listen(1)
        
        # Start listening thread
        self.socket_thread = threading.Thread(target=self.socket_listener, daemon=True)
        self.socket_thread.start()
    
    def socket_listener(self):
        """Listen for incoming connections and commands"""
        print("📡 Socket server ready on localhost:9999")
        while True:
            try:
                client, addr = self.server_socket.accept()
                print(f"✅ ROS2 bridge connected from {addr}")
                
                while True:
                    data = client.recv(1024)
                    if not data:
                        break
                    
                    # Parse JSON command
                    try:
                        cmd = json.loads(data.decode('utf-8'))
                        
                        # Base velocity commands
                        if 'linear_x' in cmd or 'angular_z' in cmd:
                            self.linear_x = cmd.get('linear_x', self.linear_x)
                            self.angular_z = cmd.get('angular_z', self.angular_z)
                        
                        # Arm joint position commands
                        if 'arm_joints' in cmd:
                            arm_positions = cmd['arm_joints']
                            if len(arm_positions) == 6:
                                self.arm_target_positions = arm_positions
                                print(f"🦾 Arm command: {[f'{p:.3f}' for p in arm_positions]}")
                            else:
                                print(f"⚠️  Invalid arm command: expected 6 joints, got {len(arm_positions)}")
                        
                    except json.JSONDecodeError as e:
                        print(f"⚠️  JSON decode error: {e}")
                
                print("❌ ROS2 bridge disconnected")
                client.close()
            except Exception as e:
                print(f"❌ Socket error: {e}")
    
    def update_base_motors(self):
        """Update wheel motor velocities based on current command"""
        # Differential drive kinematics
        left_velocity = (self.linear_x - self.angular_z * self.wheel_separation / 2.0) / self.wheel_radius
        right_velocity = (self.linear_x + self.angular_z * self.wheel_separation / 2.0) / self.wheel_radius
        
        # Set velocities
        self.front_left.setVelocity(left_velocity)
        self.rear_left.setVelocity(left_velocity)
        self.front_right.setVelocity(right_velocity)
        self.rear_right.setVelocity(right_velocity)
    
    def update_arm_motors(self):
        """Update arm joint positions based on target positions"""
        for i, motor in enumerate(self.arm_motors):
            motor.setPosition(self.arm_target_positions[i])
    
    def run(self):
        """Main control loop"""
        print("\n🚀 Controller running! Waiting for commands...")
        while self.robot.step(self.timestep) != -1:
            self.update_base_motors()
            self.update_arm_motors()

if __name__ == '__main__':
    controller = MobileManipulatorController()
    controller.run()
