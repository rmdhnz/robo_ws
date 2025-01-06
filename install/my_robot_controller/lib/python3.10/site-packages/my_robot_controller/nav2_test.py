#!/usr/bin/env python3
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations

def create_pose_stamped(navigator:BasicNavigator,position_x,position_y,orientation_z) : 
    qx,qy,qz,qw = tf_transformations.quaternion_from_euler(0.0,0.0,orientation_z)
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = position_x
    pose.pose.position.y = position_y
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose


def main() : 
    try:
        rclpy.init()
        nav = BasicNavigator()
        initial_pose = create_pose_stamped(nav,0.0,0.0,0.0)
        nav.setInitialPose(initial_pose)
        nav.waitUntilNav2Active()

        # send nav goal
        waypoints = []
        waypoints.append(create_pose_stamped(nav,2.0,-8.0,-1.57))
        waypoints.append(create_pose_stamped(nav,8.0,-3.0,1.57))
        
        # follow  waypoints
        nav.followWaypoints(waypoints)
        while not nav.isTaskComplete() : 
            feedback = nav.getFeedback()
            # print(feedback)
        
        print(nav.getResult)

        rclpy.shutdown()        
    except KeyboardInterrupt:
        print('')

if __name__ == "__main__": 
    main()