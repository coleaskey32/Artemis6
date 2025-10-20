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
    start_base_arg = DeclareLaunchArgument(
        'start_base',
        default_value='true',
        description='Start the base controller node'
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
    start_base = LaunchConfiguration('start_base')
    start_arm = LaunchConfiguration('start_arm')
    start_square = LaunchConfiguration('start_square')
    
    # Base Controller Node
    base_node = Node(
        package='robot',
        executable='base_controller',
        name='base_controller',
        output='screen',
        condition=IfCondition(start_base),
        parameters=[],
        respawn=True,  # Restart if crashes
        respawn_delay=2.0
    )
    
    # Arm Controller Node (optional)
    arm_node = Node(
        package='robot',
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
        package='robot',
        executable='square_drive',
        name='square_drive',
        output='screen',
        condition=IfCondition(start_square)
    )
    
    return LaunchDescription([
        start_base_arg,
        start_arm_arg,
        start_square_arg,
        base_node,
        arm_node,
        square_node,
    ])

