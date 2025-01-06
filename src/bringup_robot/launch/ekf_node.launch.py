from launch import LaunchDescription as Ld
from launch_ros.actions import Node
import os
# from ament_index_python.packages import get_package_share_directory

def generate_launch_description() : 
    return Ld([
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[os.path.join(
                os.path.expanduser('~/ros2_ws/src/bringup_robot/config'),'ekf_imu_config.yaml'
            )]
        )
    ])

""" 
def generate_launch_description():
    ld = LaunchDescription()
    config_file = os.path.join(
        get_package_share_directory('bringup_robot'),  # Nama package
        'config',
        'ekf.yaml'  # Path ke file ekf.yaml
    )
    mynode = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[config_file]
    )
    ld.add_action(mynode)
    return ld
""" 