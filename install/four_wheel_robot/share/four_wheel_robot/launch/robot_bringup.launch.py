#!/usr/bin/env python3
"""
Launch file for Mobile Manipulator Robot
Starts all necessary nodes for robot operation
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    # Declare launch arguments
    start_bridge_arg = DeclareLaunchArgument(
        'start_bridge',
        default_value='true',
        description='Start the ROS2-Webots bridge node'
    )
    
    start_arm_arg = DeclareLaunchArgument(
        'start_arm',
        default_value='false',
        description='Start the arm controller node'
    )
    
    start_square_arg = DeclareLaunchArgument(
        'start_square',
        default_value='false',
        description='Start the square drive demo'
    )
    
    # Get launch configuration
    start_bridge = LaunchConfiguration('start_bridge')
    start_arm = LaunchConfiguration('start_arm')
    start_square = LaunchConfiguration('start_square')
    
    # ROS2-Webots Bridge Node
    bridge_node = Node(
        package='four_wheel_robot',
        executable='ros2_bridge',
        name='webots_ros2_bridge',
        output='screen',
        condition=IfCondition(start_bridge),
        parameters=[],
        respawn=True,  # Restart if crashes
        respawn_delay=2.0
    )
    
    # Arm Controller Node (optional)
    arm_node = Node(
        package='four_wheel_robot',
        executable='arm_controller',
        name='arm_controller',
        output='screen',
        condition=IfCondition(start_arm),
        parameters=[],
        respawn=True,
        respawn_delay=2.0
    )
    
    # Square Drive Demo Node (optional)
    square_node = Node(
        package='four_wheel_robot',
        executable='square_drive',
        name='square_drive',
        output='screen',
        condition=IfCondition(start_square)
    )
    
    return LaunchDescription([
        start_bridge_arg,
        start_arm_arg,
        start_square_arg,
        bridge_node,
        arm_node,
        square_node,
    ])

