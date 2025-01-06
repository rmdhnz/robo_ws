import time
import board
import busio
from adafruit_bno055 import BNO055_I2C
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class BNO055Node(Node):
    def __init__(self):
        super().__init__('bno055_node')
        self.publisher_ = self.create_publisher(Float32, 'bno_data', 100)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

        # Inisialisasi I2C dan BNO055
        i2c = busio.I2C(board.SCL, board.SDA)
        self.sensor = BNO055_I2C(i2c)

        # Variabel global
        self.yaw_raw1 = 0.0
        self.yaw_raw2 = 0.0
        self.yaw = 0.0

        # Inisialisasi sensor
        self.get_logger().info("Orientation Sensor Test")
        if self.sensor.temperature is None:
            self.get_logger().error("Ooops, no BNO055 detected... Check your wiring or I2C ADDR!")
            exit()

        self.get_logger().info("Wait Calibration")
        time.sleep(0.5)

        # Kalibrasi awal
        self.calibrate_sensor()

    def calibrate_sensor(self):
        yaw_raw0 = 0.0
        for i in range(200):
            yaw_raw0 = self.sensor.euler[0]  # Membaca yaw dari sensor (Euler angle)
            if yaw_raw0 is None:
                yaw_raw0 = 0.0
            self.yaw_raw1 += yaw_raw0
            time.sleep(0.01)

        self.yaw_raw1 /= 200  # Rata-rata nilai awal yaw
        self.get_logger().info(f"Calibration Offset: {self.yaw_raw1}")
        time.sleep(1)

    def timer_callback(self):
        self.yaw_raw2 = self.sensor.euler[0]
        if self.yaw_raw2 is None:
            self.yaw_raw2 = 0.0

        # Hitung yaw berdasarkan offset kalibrasi
        self.yaw = self.yaw_raw2 - self.yaw_raw1
        if self.yaw < 0:
            self.yaw += 360  # Adjust for negative angles to ensure clockwise direction

        self.get_logger().info(f"Yaw Raw: {self.yaw_raw2:.2f}, Adjusted Yaw: {self.yaw:.2f}")

        # Publish yaw data
        msg = Float32()
        msg.data = self.yaw
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = BNO055Node()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()