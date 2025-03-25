import time

import rclpy
import serial
from geometry_msgs.msg import Twist
from rclpy import Node

ser = serial.Serial("/dev/ttyACM0", 9600, timeout=1)  # change serial name to arduino


# define the subscriber node class to subscriber from twist_pub
class SerialTwistPass(Node):

    def __init__(self):
        super().__init__("serial_twist_pass")
        self.subscription = self.create_subscription(
            Twist, "desired_twist", self.serial_parse, 10
        )
        self.subscription  # prevent unused variable warning

    # define the subscriber callback to receive data, parse it into the desired format, and send it
    # to the arduino over serial
    def serial_parse(self, msg):
        global ser

        # parse the data for serial into the proper format
        lin_speed = str(msg.linear.x)
        ang_speed = str(msg.angular.z)

        motor_command = lin_speed + ":" + ang_speed + "\n"

        # record the received message on the console
        self.get_logger().info(motor_command)

        ser.reset_input_buffer()  # flush input buffer, discarding all its contents
        ser.write(motor_command.encode("utf-8"))

        # send the data to the arduino over serial


def main(args=None):
    rclpy.init(args=args)

    serial_twist_pass = SerialTwistPass

    rclpy.spin(serial_twist_pass)

    serial_twist_pass.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
