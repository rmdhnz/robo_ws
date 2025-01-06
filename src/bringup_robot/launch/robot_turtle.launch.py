from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description() : 
    try : 
        ld = LaunchDescription()
        turtle_node = Node(
            package='turtlesim',
            executable='turtlesim_node',
        )
        controller_node = Node(
            package="my_robot_controller",
            executable="turtle_controller"
        )
        pose_node = Node(
            package="my_robot_controller",
            executable="pose_subscriber",
        )
        ld.add_action(turtle_node)
        ld.add_action(controller_node)
        ld.add_action(pose_node)
        return ld
    except KeyboardInterrupt : 
        print("")