#!/usr/bin/env python3
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations
from rclpy.node import Node
from std_msgs.msg import String
class NavTest(Node) : 
    def __init__(self):
        super().__init__('nav_test_node')
        self.get_logger().info('Navigation test has been started...')
        self.pub = self.create_publisher(String,'info_nav',10)
        self.navigation_start()
    
    def navigation_start(self) : 
        msg = String()
        nav = BasicNavigator()
        qx,qy,qz,qw = tf_transformations.quaternion_from_euler(0.0,0.0,0.0)
        initial_pose = PoseStamped()
        initial_pose.header.frame_id ='map'
        initial_pose.header.stamp = nav.get_clock().now().to_msg()
        initial_pose.pose.position.x = 0.0
        initial_pose.pose.position.y = 0.0
        initial_pose.pose.position.z = 0.0
        initial_pose.pose.orientation.x = qx
        initial_pose.pose.orientation.y = qy
        initial_pose.pose.orientation.z = qz
        initial_pose.pose.orientation.w = qw
        nav.setInitialPose(initial_pose)
        nav.waitUntilNav2Active()

        goal_pose = PoseStamped()
        goal_pose.header.frame_id ='map'
        goal_pose.header.stamp = nav.get_clock().now().to_msg()
        goal_pose.pose.position.x = 10.0
        goal_pose.pose.position.y = -3.0
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = qx
        goal_pose.pose.orientation.y = qy
        goal_pose.pose.orientation.z = qz
        goal_pose.pose.orientation.w = qw
        nav.goToPose(goal_pose)
        while not nav.isTaskComplete() : 
            feedback = nav.getFeedback()
            msg.data = "Proccess"
            self.pub.publish(msg)
            self.get_logger().info('Robot is on the way to the goal')
            # print(feedback)
        print(nav.getResult())
        msg.data  = 'Success'
        self.pub.publish(msg)
        self.get_logger().info('Navigation has been finished!')
        print(type(nav.getResult()))



def main() : 
    rclpy.init()
    node = NavTest()
    rclpy.shutdown()

if __name__ == '__main__':
    main()