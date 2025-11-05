import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial

class ArduinoEncoderReader(Node):
    def __init__(self):
        super().__init__('arduino_encoder_reader')

        # Publisher ROS2
        self.publisher_ = self.create_publisher(Float32MultiArray, 'encoder_counter', 10)

        # Serial setup
        self.ser = serial.Serial('/dev/ttyACM1', 115200, timeout=1)

        # Timer per leggere seriale ogni 0.02 s (50 Hz)
        self.timer = self.create_timer(0.01, self.read_serial)

    def read_serial(self):
        #if self.ser.in_waiting > 0:
        try:
            line = self.ser.readline().decode('utf-8').strip()
            if line:
                parts = line.split()
                if len(parts) == 2:
                    msg = Float32MultiArray()
                    msg.data = [float(parts[0]), float(parts[1])]
                    self.publisher_.publish(msg)
                    #self.get_logger().info(f"Pubblicato: {msg.data}")
        except Exception as e:
            pass
            #self.get_logger().error(f"Errore lettura seriale: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoEncoderReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
