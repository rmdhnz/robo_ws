from launch import LaunchDescription as Ld
from launch_ros.actions import Node

def generate_launch_description() : 
    ld = Ld()
    imu_node = Node(
        package="my_robot_controller",
        executable="imu_raw_data",
    )
    odom_node = Node(
        package="my_robot_controller",
        executable="odom_raw_data",
    )
    fusion_node = Node(
        package="my_robot_controller",
        executable="ekf_fusion_node",
    )
    ld.add_action(imu_node)
    ld.add_action(odom_node)
    ld.add_action(fusion_node)
    return ld