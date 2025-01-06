#!/usr/bin/env python3


from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '/path/to/omni4wd_world.sdf'],
            output='screen'
        ),
        Node(
            package='controller_manager',
            executable='spawner.py',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        ),
        Node(
            package='controller_manager',
            executable='spawner.py',
            arguments=['forward_velocity_controller', '--controller-manager', '/controller_manager'],
        )
    ])
