#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    timeout_arg = DeclareLaunchArgument('timeout')
    
    timer_node = Node(
        package='uuv_assistants',
        executable='set_simulation_timer.py',
        name='simulation_timeout',
        output='screen',
        parameters=[{'timeout': LaunchConfiguration('timeout')}]
    )
    
    return LaunchDescription([timeout_arg, timer_node])
