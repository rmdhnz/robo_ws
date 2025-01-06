from launch import LaunchDescription as Ld
from launch_ros.actions import Node
import os

def generate_launch_description():
    return Ld([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(
                os.path.expanduser('~/ros2_ws/src/bringup_robot/config'), 'ekf_lidar_imu_odom.yaml'
            )]
        )
    ])