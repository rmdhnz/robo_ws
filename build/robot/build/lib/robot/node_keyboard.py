import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import sys
import select
import termios
import tty

class KeyboarNode(Node):
    def __init__(self):
        super().__init__('keyboard_node')
        self.subscription = self.create_subscription(
            String,
            'arduino_data',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(String, 'user_command', 10)
        self.subscription 
        self.get_logger().info('keyboard_node initialized')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received from Arduino: {msg.data}')

    def send_command(self, command):
        msg = String()
        msg.data = command
        self.publisher_.publish(msg)

def user_input_thread(node, speed):
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    try:
        while rclpy.ok():
            if select.select([sys.stdin], [], [], 1)[0]:
                user_input = sys.stdin.read(1)
                if user_input:
                    command = f"{user_input}{speed}"
                    node.send_command(command)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboarNode()

    try:
        speed_percentage = int(input("Enter speed percentage (0-100): "))
        if speed_percentage < 0 or speed_percentage > 100:
            raise ValueError("Speed percentage must be between 0 and 100")
        speed = int((speed_percentage / 100) * 500)
    except ValueError as e:
        print(e)
        return

    input_thread = threading.Thread(target=user_input_thread, args=(node, speed))
    input_thread.daemon = True
    input_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()