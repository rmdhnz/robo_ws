import csv
import signal
import sys
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomSaver(Node):
    def __init__(self):
        super().__init__('odom_saver')
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.data = []

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = msg.pose.pose.position.z  # Assuming yaw is stored in orientation.z

        self.data.append((x, y, yaw))

    def save_data(self, filename='data_odom.csv'):
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['x', 'y', 'yaw'])
            writer.writerows(self.data)

def signal_handler(sig, frame):
    node.save_data()
    rclpy.shutdown()
    sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    global node
    node = OdomSaver()
    signal.signal(signal.SIGINT, signal_handler)
    rclpy.spin(node)

if __name__ == '__main__':
    main()