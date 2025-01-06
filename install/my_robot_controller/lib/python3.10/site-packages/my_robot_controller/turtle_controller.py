#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen
from functools import partial
import matplotlib.pyplot as plt
class TurtleController(Node) : 
    def __init__(self):
        super().__init__("turtle_controller")
        self.prev_x = 0
        self.coord_x = []
        self.coord_y = []
        self.cmd_vel_publisher = self.create_publisher(Twist,"/turtle1/cmd_vel",10)
        self.pose_subscriber = self.create_subscription(Pose,"/turtle1/pose",self.pose_callback,10)
        self.get_logger().info("TURTLE CONTROL has been started and changed....")

    def pose_callback(self,pose:Pose) : 
        cmd = Twist()
        self.coord_x.append(pose.x)
        self.coord_y.append(pose.y)
        if pose.x >9.0 or pose.x < 2.0 or pose.y >9.0 or pose.y <2.0: 
            cmd.linear.x = 1.0
            cmd.angular.z = 0.9
        else : 
            cmd.linear.x = 5.0
            cmd.angular.z = 0.0
        self.cmd_vel_publisher.publish(cmd)

        if pose.x > 5.5 and self.prev_x <= 5.5: 
            self.prev_x = pose.x
            self.get_logger().info("Set color to red")
            self.call_set_pen_service(255,0,0,3,0)
        elif pose.x < 5.5 and self.prev_x > 5.5 : 
            self.prev_x = pose.x
            self.get_logger().info("Set color to green")
            self.call_set_pen_service(0,255,0,3,0)
    
    def call_set_pen_service(self,r,g,b,width,off) : 
        client = self.create_client(SetPen,"/turtle1/set_pen")
        while not client.wait_for_service(1.0) : 
            self.get_logger().warn("Waiting for service")
        
        req = SetPen.Request()
        req.r = r
        req.b=b
        req.g=g
        req.width=width
        req.off=off

        future = client.call_async(req)
        future.add_done_callback(partial(self.callback_set_pen))
    
    def callback_set_pen(self,future) : 
        try :
            response = future.result()
        except Exception as e :
            self.get_logger().error("Service error : ",e)



def main(args=None): 
    try : 
        rclpy.init(args=args)
        node = TurtleController()
        rclpy.spin(node)
        rclpy.shutdown()
    except KeyboardInterrupt :
        plt.plot(node.coord_x,node.coord_y)
        plt.show()
        print("Thanks")