from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description() : 
    ld = LaunchDescription()
    imu_node = Node(
        package="my_robot_controller",
        executable="imu_node",
    )
    imu_filtered_node  = Node(
        package="my_robot_controller",
        executable="imu_filtered_node",
    )
    ld.add_action(imu_node)
    ld.add_action(imu_filtered_node)
    return ld
