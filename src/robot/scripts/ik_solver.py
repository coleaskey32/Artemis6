#!/usr/bin/env python3
"""
IK Solver Node - Inverse Kinematics for the 6-axis arm
Subscribes to target gripper poses and publishes joint commands
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
import numpy as np
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import ikpy.utils.plot as plot_utils

class IKSolver(Node):
    def __init__(self):
        super().__init__('ik_solver')
        
        # Create kinematic chain based on our robot arm
        # Link lengths from robot.wbt
        self.chain = Chain(name='robot_arm', links=[
            OriginLink(),  # Base (fixed)
            URDFLink(
                name="shoulder_pan",
                origin_translation=[0, 0, 0.06],  # Base height
                origin_orientation=[0, 0, 0],
                rotation=[0, 0, 1],  # Rotation around Z
                bounds=(-6.28, 6.28)
            ),
            URDFLink(
                name="shoulder_lift",
                origin_translation=[0, 0, 0.06],  # Shoulder offset
                origin_orientation=[0, 0, 0],
                rotation=[0, 1, 0],  # Rotation around Y
                bounds=(-3.14, 3.14)
            ),
            URDFLink(
                name="elbow",
                origin_translation=[0, 0, 0.375],  # Upper arm length
                origin_orientation=[0, 0, 0],
                rotation=[0, 1, 0],  # Rotation around Y
                bounds=(-3.14, 3.14)
            ),
            URDFLink(
                name="wrist_1",
                origin_translation=[0, 0, 0.285],  # Forearm length
                origin_orientation=[0, 0, 0],
                rotation=[0, 0, 1],  # Rotation around Z
                bounds=(-6.28, 6.28)
            ),
            URDFLink(
                name="wrist_2",
                origin_translation=[0, 0, 0.05],  # Wrist 1 offset
                origin_orientation=[0, 0, 0],
                rotation=[0, 1, 0],  # Rotation around Y
                bounds=(-6.28, 6.28)
            ),
            URDFLink(
                name="wrist_3",
                origin_translation=[0, 0, 0.02],  # Wrist 2 offset
                origin_orientation=[0, 0, 0],
                rotation=[0, 0, 1],  # Rotation around Z
                bounds=(-6.28, 6.28)
            ),
        ])
        
        # Subscriber for target poses
        self.subscription = self.create_subscription(
            PoseStamped,
            '/gripper_target_pose',
            self.pose_callback,
            10
        )
        
        # Publisher for joint commands
        self.publisher = self.create_publisher(
            JointState,
            '/arm_joint_commands',
            10
        )
        
        # Current joint state (start at home)
        self.current_joints = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # 7 values (includes base origin)
        
        self.get_logger().info("🧮 IK Solver Started!")
        self.get_logger().info("📡 Listening to /gripper_target_pose")
        self.get_logger().info("📤 Publishing to /arm_joint_commands")
        self.get_logger().info("")
        self.get_logger().info("🎯 Test with:")
        self.get_logger().info('  ros2 topic pub /gripper_target_pose geometry_msgs/msg/PoseStamped \\')
        self.get_logger().info('    "{pose: {position: {x: 0.3, y: 0.0, z: 0.3}}}"')
        
    def pose_callback(self, msg):
        """Solve IK for target pose and publish joint commands"""
        # Extract target position
        target_position = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ]
        
        # Extract target orientation (quaternion)
        target_orientation = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ]
        
        self.get_logger().info(f"🎯 Target: x={target_position[0]:.3f}, y={target_position[1]:.3f}, z={target_position[2]:.3f}")
        
        try:
            # Build target transformation matrix
            target_frame = np.eye(4)
            target_frame[:3, 3] = np.array(target_position)
            
            # Solve IK using inverse_kinematics_frame method
            joint_angles = self.chain.inverse_kinematics_frame(
                target=target_frame
            )
            
            # Update current state
            self.current_joints = joint_angles
            
            # Extract joint values (skip first element which is the base origin)
            arm_joints = joint_angles[1:7]  # Joints 1-6
            
            # Publish joint commands
            joint_msg = JointState()
            joint_msg.position = arm_joints.tolist()
            self.publisher.publish(joint_msg)
            
            # Log the solution
            joint_str = ', '.join([f'{j:.3f}' for j in arm_joints])
            self.get_logger().info(f"✅ Solution: [{joint_str}]")
            
            # Calculate and show forward kinematics (verification)
            fk_frame = self.chain.forward_kinematics(joint_angles)
            fk_position = fk_frame[:3, 3]
            error = np.linalg.norm(fk_position - target_position)
            self.get_logger().info(f"📍 Achieved: x={fk_position[0]:.3f}, y={fk_position[1]:.3f}, z={fk_position[2]:.3f}")
            self.get_logger().info(f"📏 Error: {error*1000:.2f}mm")
            
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

