#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import tf_transformations
import numpy as np
class OdomFiltered(Node):
    def __init__(self):
        super().__init__('odom_filtered_node')
        self.get_logger().info('Odom filtered has been started...')
        self.subs = self.create_subscription(
            Odometry,
            "/odometry/filtered",
            self.callback_odom,
            10
        )
    
    def callback_odom(self,msg:Odometry) : 
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        quaternion = np.array([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])
        roll,pitch,yaw = tf_transformations.euler_from_quaternion(quaternion)
        self.get_logger().info(f"\nX : {x:.2f}\nY : {y:.2f}\nYaw : {yaw:.2f}\n")

def main(args=None):
    try:
        rclpy.init(args=args)
        node = OdomFiltered()
        rclpy.spin(node)
        rclpy.shutdown()
    except KeyboardInterrupt:
        print('Thanks')