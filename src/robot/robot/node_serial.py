import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
import serial
import time
import threading

class IO_Arduino(Node):
    def __init__(self):
        super().__init__('io_arduino')
        self.publisher_ = self.create_publisher(String, 'arduino_data', 10) 
        self.wheel_twist_publisher = self.create_publisher(Float32MultiArray, 'wheel_twist', 10) # data rpm roda
        self.subscription = self.create_subscription(
            String,
            'joy_command',
            self.listener_callback, #sub data keceptan
            10)
        self.subscription  
        self.serial_port = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AI05UAHH-if00-port0' #pake ini klo tty dia bisa ganti2 antara arduino sama lidarnya
        self.baud_rate = 115200
        self.ser = None
        self.serial_connected = False
        self.connect_serial_thread = threading.Thread(target=self.connect_serial)
        self.connect_serial_thread.daemon = True
        self.connect_serial_thread.start()
        self.timer = self.create_timer(1.0 / 50.0, self.timer_callback)
        self.get_logger().info('Serial node initialized')

    def connect_serial(self):
        while True:
            if self.ser is None or not self.ser.is_open:
                try:
                    self.ser = serial.Serial(self.serial_port, self.baud_rate)
                    self.serial_connected = True
                    self.get_logger().info(f'Connected to serial port: {self.serial_port}')
                except serial.SerialException:
                    self.serial_connected = False
                    self.get_logger().error(f'Failed to connect to serial port: {self.serial_port}. Retrying...')
                    time.sleep(1)
            time.sleep(1)

    def timer_callback(self):
        if not self.serial_connected:
            self.get_logger().error('Serial connection lost. Attempting to reconnect...')
            return

        try:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8').strip()
                msg = String()
                msg.data = line
                self.publisher_.publish(msg)
                self.get_logger().info(f'Received from serial: {line}')
                
                parts = line.split(' ')
                if len(parts) == 4 and all(self.is_valid_integer(part) for part in parts):
                    speeds = list(map(float, parts)) 
                    twist_msg = Float32MultiArray()
                    twist_msg.data = speeds
                    self.wheel_twist_publisher.publish(twist_msg)

        except UnicodeDecodeError:
            self.get_logger().error('Failed to decode line')
        except serial.SerialException:
            self.get_logger().error('Serial connection lost. Attempting to reconnect...')
            self.ser = None
            self.serial_connected = False
            if not self.connect_serial_thread.is_alive():
                self.connect_serial_thread = threading.Thread(target=self.connect_serial)
                self.connect_serial_thread.daemon = True
                self.connect_serial_thread.start()
        except OSError as e:
            self.get_logger().error(f'OS error: {e}')
            self.ser = None
            self.serial_connected = False
            if not self.connect_serial_thread.is_alive():
                self.connect_serial_thread = threading.Thread(target=self.connect_serial)
                self.connect_serial_thread.daemon = True
                self.connect_serial_thread.start()

    def is_valid_integer(self, s):
        try:
            int(s)
            return True
        except ValueError:
            return False

    def listener_callback(self, msg):
        if not self.serial_connected:
            self.get_logger().error('Serial connection lost. Cannot send joy command.')
            return

        try:
            self.ser.write((msg.data + '\n').encode('utf-8'))
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write error: {e}')
            self.ser = None
            self.serial_connected = False
            if not self.connect_serial_thread.is_alive():
                self.connect_serial_thread = threading.Thread(target=self.connect_serial)
                self.connect_serial_thread.daemon = True
                self.connect_serial_thread.start()
        except OSError as e:
            self.get_logger().error(f'OS error: {e}')
            self.ser = None
            self.serial_connected = False
            if not self.connect_serial_thread.is_alive():
                self.connect_serial_thread = threading.Thread(target=self.connect_serial)
                self.connect_serial_thread.daemon = True
                self.connect_serial_thread.start()

def main(args=None):
    rclpy.init(args=args)
    node = IO_Arduino()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()