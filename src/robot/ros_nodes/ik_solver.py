#!/usr/bin/env python3
"""
IK Solver Node - Inverse Kinematics for 5-DOF Arm
Subscribes to target gripper poses and publishes joint commands

Arm Configuration (from URDF):
  Joint 1: base_yaw     - Z rotation (±3.14 rad)
  Joint 2: shoulder     - Y rotation (±1.57 rad)  
  Joint 3: elbow        - Y rotation (0 to 1.57 rad)
  Joint 4: wrist_pitch  - Y rotation (±1.57 rad)
  Joint 5: wrist_roll   - Z rotation (±3.14 rad)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty
import numpy as np
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink

class IKSolver(Node):
    def __init__(self):
        super().__init__('ik_solver')
        
        # Create kinematic chain matching URDF dimensions
        self.chain = Chain(name='robot_arm', links=[
            OriginLink(),
            URDFLink(
                name="base",
                origin_translation=[0, 0, 0.10],
                origin_orientation=[0, 0, 0],
                rotation=[0, 0, 1],
                bounds=(-3.14159, 3.14159)
            ),
            URDFLink(
                name="shoulder",
                origin_translation=[0, 0, 0.18],
                origin_orientation=[0, 0, 0],
                rotation=[0, 1, 0],
                bounds=(-1.5708, 1.5708)
            ),
            URDFLink(
                name="elbow",
                origin_translation=[0, 0, 0.20],
                origin_orientation=[0, 0, 0],
                rotation=[0, 1, 0],
                bounds=(0.0, 1.5708)
            ),
            URDFLink(
                name="wrist_pitch",
                origin_translation=[0, 0, 0.15],
                origin_orientation=[0, 0, 0],
                rotation=[0, 1, 0],
                bounds=(-1.5708, 1.5708)
            ),
            URDFLink(
                name="wrist_roll",
                origin_translation=[0, 0, 0.1775],
                origin_orientation=[0, 0, 0],
                rotation=[0, 0, 1],
                bounds=(-3.14159, 3.14159)
            ),
            URDFLink(
                name="gripper",
                origin_translation=[0, 0, 0.11],
                origin_orientation=[0, 0, 0],
                rotation=[0, 0, 0],
            ),
        ])
        
        # Subscribers
        self.subscription = self.create_subscription(
            Point, '/arm_coordinates', self.target_callback, 10)
        self.home_subscription = self.create_subscription(
            Empty, '/arm_home', self.home_callback, 10)
        
        # Publisher for joint commands (5 DOF)
        self.joint_publisher = self.create_publisher(JointState, '/arm_joint_commands', 10)
        
        # HOME position - safe upright pose [base_yaw, shoulder, elbow, wrist_pitch, wrist_roll]
        self.HOME_JOINTS = [0.0, 0.785, 0.785, 0.0, 0.0]
        
        # Current joint state (IKPy needs 7 values: origin + 5 joints + end effector)
        self.current_joints = np.zeros(7)
        
        # Pending target for delayed execution
        self.pending_target = None
        self.delay_timer = None
        
        # Time to wait at home before moving to target (seconds)
        self.home_delay = 1.5
        
        # Workspace bounds
        self.max_reach = 0.10 + 0.18 + 0.20 + 0.15 + 0.1775 + 0.11
        self.min_reach = 0.15
        
        self.get_logger().info("🧮 IK Solver Started!")
        self.get_logger().info(f"📏 Arm reach: {self.min_reach:.2f}m to {self.max_reach:.2f}m")
        self.get_logger().info(f"🏠 Home-first enabled (delay: {self.home_delay}s)")
        self.get_logger().info("")
        self.get_logger().info("📡 Topics:")
        self.get_logger().info("   /arm_coordinates (Point) - Move to XYZ")
        self.get_logger().info("   /arm_home (Empty) - Go to home position")
        self.get_logger().info("")
        self.get_logger().info("🎯 Test:")
        self.get_logger().info('  ros2 topic pub -1 /gripper_target geometry_msgs/msg/Point "{x: 0.3, y: 0.0, z: 0.4}"')
    
    def publish_joints(self, joint_positions):
        """Publish joint positions to arm controller."""
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = ['base_yaw', 'shoulder', 'elbow', 'wrist_pitch', 'wrist_roll']
        joint_msg.position = joint_positions
        self.joint_publisher.publish(joint_msg)
    
    def go_home(self):
        """Send arm to home position."""
        self.get_logger().info(f"🏠 Going HOME: {self.HOME_JOINTS}")
        self.publish_joints(self.HOME_JOINTS)
        # Update IKPy's current state
        self.current_joints = np.array([0] + self.HOME_JOINTS + [0])
    
    def home_callback(self, msg):
        """Handle manual home command."""
        self.go_home()
    
    def target_callback(self, msg):
        """Handle target position - goes home first, then to target."""
        self.get_logger().info(f"🎯 New target: x={msg.x:.3f}, y={msg.y:.3f}, z={msg.z:.3f}")
        
        # Cancel any pending motion
        if self.delay_timer:
            self.delay_timer.cancel()
            self.delay_timer = None
        
        # Step 1: Go to HOME first
        self.go_home()
        
        # Step 2: Store target and wait for arm to reach home
        self.pending_target = np.array([msg.x, msg.y, msg.z])
        self.delay_timer = self.create_timer(self.home_delay, self.execute_pending_target)
    
    def execute_pending_target(self):
        """Called after delay - solve IK and move to target."""
        # Cancel the timer (one-shot)
        if self.delay_timer:
            self.delay_timer.cancel()
            self.delay_timer = None
        
        if self.pending_target is None:
            return
        
        target = self.pending_target
        self.pending_target = None
        
        self.get_logger().info(f"🎯 Moving to target: x={target[0]:.3f}, y={target[1]:.3f}, z={target[2]:.3f}")
        
        # Check reachability
        distance = np.linalg.norm(target)
        if distance > self.max_reach:
            self.get_logger().warn(f"⚠️  Target may be out of reach ({distance:.2f}m > {self.max_reach:.2f}m)")
        
        try:
            # Use home position as initial guess (we're starting from home)
            initial_guess = np.array([0] + self.HOME_JOINTS + [0])
            
            # Solve IK
            joint_angles = self.chain.inverse_kinematics(
                target_position=target,
                initial_position=initial_guess
            )
            
            # Update current state
            self.current_joints = joint_angles
            
            # Extract 5 joint values (skip origin [0] and end effector [6])
            arm_joints = joint_angles[1:6].tolist()
            
            # Publish to arm
            self.publish_joints(arm_joints)
            
            # Log solution
            joint_str = ', '.join([f'{j:.3f}' for j in arm_joints])
            self.get_logger().info(f"✅ Solution: [{joint_str}]")
            
            # Verify with forward kinematics
            fk_frame = self.chain.forward_kinematics(joint_angles)
            achieved = fk_frame[:3, 3]
            error = np.linalg.norm(achieved - target)
            
            self.get_logger().info(f"📍 Achieved: x={achieved[0]:.3f}, y={achieved[1]:.3f}, z={achieved[2]:.3f}")
            self.get_logger().info(f"📏 Error: {error*1000:.1f}mm")
            
        except Exception as e:
            self.get_logger().error(f"❌ IK failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    solver = IKSolver()
    
    try:
        rclpy.spin(solver)
    except KeyboardInterrupt:
        solver.get_logger().info("👋 Shutting down IK Solver")
    finally:
        solver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
