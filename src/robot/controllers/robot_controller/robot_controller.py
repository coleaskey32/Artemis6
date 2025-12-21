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
        print("   ├─ 🦾 5-DOF Robotic Arm: Ready")
        print("   └─ 🤏 Prismatic Gripper: Ready")
        print()
        print("📡 Listening for ROS2 commands on port 9999")
        print("   ├─ Base control: /cmd_vel  (via base_controller)")
        print("   ├─ Arm control: /arm_joint_commands  (via arm_controller)")
        print("   └─ Gripper control: /gripper_command  (via arm_controller)")
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
        # Get wheel motor devices (names from proto file)
        self.front_left = self.robot.getDevice('front_left_wheel_joint')
        self.front_right = self.robot.getDevice('front_right_wheel_joint')
        self.rear_left = self.robot.getDevice('back_left_wheel_joint')
        self.rear_right = self.robot.getDevice('back_right_wheel_joint')
        
        # Check if all motors were found
        for name, motor in [
            ('front_left_wheel_joint', self.front_left),
            ('front_right_wheel_joint', self.front_right),
            ('back_left_wheel_joint', self.rear_left),
            ('back_right_wheel_joint', self.rear_right)
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
        """Set up the robotic arm motors and sensors (5-DOF + gripper)"""
        # Get arm joint motors (5-DOF)
        self.arm_motors = []
        self.arm_sensors = []
        
        arm_joint_names = [
            'base_yaw_joint',
            'shoulder_joint',
            'elbow_joint',
            'wrist_pitch_joint',
            'wrist_roll_joint'
        ]
        
        for name in arm_joint_names:
            motor = self.robot.getDevice(name)
            sensor = self.robot.getDevice(f'{name}_sensor')
            
            if motor is None:
                self.get_logger().error(f"❌ Motor '{name}' not found!")
                raise RuntimeError(f"Motor '{name}' not found")
            
            if sensor is None:
                self.get_logger().error(f"❌ Sensor '{name}_sensor' not found!")
                raise RuntimeError(f"Sensor '{name}_sensor' not found")
            
            # Enable position sensors
            sensor.enable(self.timestep)
            
            # Set motors to position control mode (default)
            # Initialize to home position (all zeros)
            motor.setPosition(0.0)
            
            self.arm_motors.append(motor)
            self.arm_sensors.append(sensor)
        
        # Target arm joint positions (radians)
        self.arm_target_positions = [0.0] * 5
        
        # Setup gripper (both prismatic joints)
        self.gripper_motor1 = self.robot.getDevice('gripper_joint')
        self.gripper_motor2 = self.robot.getDevice('gripper2_joint')
        
        if self.gripper_motor1 is None:
            self.get_logger().error("❌ Gripper motor 'gripper_joint' not found!")
            raise RuntimeError("Gripper motor 'gripper_joint' not found")
        
        if self.gripper_motor2 is None:
            self.get_logger().warn("⚠️  Gripper motor 'gripper2_joint' not found - using single gripper")
            self.gripper_motor2 = None
        
        # Start closed
        self.gripper_motor1.setPosition(0.0)
        if self.gripper_motor2:
            self.gripper_motor2.setPosition(0.0)
        
        self.gripper_target_position = 0.0  # meters (0.0 = closed, 0.03 = open)
    
    def setup_socket_server(self):
        """Set up socket server in a separate thread"""
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind(('localhost', 9999))
        self.server_socket.listen(5)  # Allow multiple pending connections
        
        # Track connected clients
        self.connected_clients = 0
        
        # Start listening thread
        self.socket_thread = threading.Thread(target=self.socket_listener, daemon=True)
        self.socket_thread.start()
    
    def socket_listener(self):
        """Listen for incoming connections and spawn handler threads"""
        print("📡 Socket server ready on localhost:9999")
        while True:
            try:
                client, addr = self.server_socket.accept()
                self.connected_clients += 1
                print(f"✅ ROS2 client #{self.connected_clients} connected from {addr}")
                
                # Handle each client in a separate thread
                client_thread = threading.Thread(
                    target=self.handle_client, 
                    args=(client, addr),
                    daemon=True
                )
                client_thread.start()
                
            except Exception as e:
                print(f"❌ Socket accept error: {e}")
    
    def handle_client(self, client, addr):
        """Handle commands from a single client connection"""
        try:
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
                    
                    # Arm joint position commands (5 DOF)
                    if 'arm_joints' in cmd:
                        arm_positions = cmd['arm_joints']
                        if len(arm_positions) == 5:
                            self.arm_target_positions = arm_positions
                            print(f"🦾 Arm: {[f'{p:.3f}' for p in arm_positions]}")
                        else:
                            print(f"⚠️  Invalid arm command: expected 5 joints, got {len(arm_positions)}")
                    
                    # Gripper position command (prismatic)
                    if 'gripper' in cmd:
                        gripper_pos = cmd['gripper']
                        self.gripper_target_position = gripper_pos
                        print(f"🤏 Gripper: {gripper_pos:.4f} m")
                    
                except json.JSONDecodeError as e:
                    print(f"⚠️  JSON decode error: {e}")
        
        except Exception as e:
            print(f"❌ Client error: {e}")
        finally:
            self.connected_clients -= 1
            print(f"❌ Client {addr} disconnected ({self.connected_clients} remaining)")
            client.close()
    
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
        """Update arm joint positions and gripper based on target positions"""
        # Update 5 DOF arm joints
        for i, motor in enumerate(self.arm_motors):
            motor.setPosition(self.arm_target_positions[i])
        
        # Update BOTH gripper fingers (mimic behavior)
        self.gripper_motor1.setPosition(self.gripper_target_position)
        self.gripper_motor2.setPosition(self.gripper_target_position)
    
    def run(self):
        """Main control loop"""
        print("\n🚀 Controller running! Waiting for commands...")
        while self.robot.step(self.timestep) != -1:
            self.update_base_motors()
            self.update_arm_motors()

if __name__ == '__main__':
    controller = MobileManipulatorController()
    controller.run()
