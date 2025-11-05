#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
import serial

class ArduinoEncoderReader:
    def __init__(self):
        # Inizializzazione del nodo
        rospy.init_node('arduino_encoder_reader', anonymous=True)

        # Publisher ROS1
        self.publisher = rospy.Publisher('encoder_counter', Float32MultiArray, queue_size=10)

        # Porta seriale
        port = rospy.get_param("~port", "/dev/ttyACM1")
        baud = rospy.get_param("~baud", 115200)

        try:
            self.ser = serial.Serial(port, baud, timeout=0.05)
            rospy.loginfo(f"Seriale aperta su {port} a {baud} baud")
        except serial.SerialException as e:
            rospy.logerr(f"Errore apertura seriale: {e}")
            rospy.signal_shutdown("Seriale non disponibile")
            return

        # Frequenza di lettura (50 Hz)
        self.rate = rospy.Rate(200)

    def read_serial(self):
        while not rospy.is_shutdown():
            #if self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                self.ser.reset_input_buffer()
                #print(line)
                if line:
                    parts = line.split()
                    if len(parts) == 2:
                        msg = Float32MultiArray()
                        msg.data = [float(parts[0]), float(parts[1])]
                        self.publisher.publish(msg)
                        #rospy.loginfo(f"Pubblicato: {msg.data}")
            except Exception as e:
                pass
                #rospy.logerr(f"Errore lettura seriale: {e}")
                
            self.rate.sleep()

    def close(self):
        if self.ser.is_open:
            self.ser.close()
            rospy.loginfo("Seriale chiusa")

if __name__ == '__main__':
    try:
        reader = ArduinoEncoderReader()
        reader.read_serial()
    except rospy.ROSInterruptException:
        pass
    finally:
        try:
            reader.close()
        except:
            pass

