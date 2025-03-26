import rclpy
import serial
from geometry_msgs.msg import Twist
from rclpy.node import Node


# define the subscriber node class to subscriber from twist_pub
class SerialTwistPass(Node):

    def __init__(self):
        super().__init__("serial_twist_pass")
        self.subscription = self.create_subscription(
            Twist, "desired_twist", self.serial_parse, 10
        )
        self.subscription  # prevent unused variable warning
        self.ser = serial.Serial(
            "/dev/ttyACM0", 57600, timeout=1
        )  # change serial name to arduino

    # define the subscriber callback to receive data, parse it into the desired format, and send it
    # to the arduino over serial
    def serial_parse(self, msg):
        # parse the data for serial into the proper format
        lin_speed = str(msg.linear.x)
        ang_speed = str(msg.angular.z)

        motor_command = lin_speed + ":" + ang_speed + "\n"

        # record the received message on the console
        # self.get_logger().info(motor_command)

        self.ser.reset_input_buffer()  # flush input buffer, discarding all its contents
        self.ser.write(motor_command.encode("utf-8"))
        line = self.ser.readline().decode('utf-8').rstrip()
        self.get_logger().info(line)

        # send the data to the arduino over serial

    def destroy(self):
        self.ser.close()  #  Close serial port before shutdown
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    serial_twist_pass = SerialTwistPass()

    try:
        rclpy.spin(serial_twist_pass)
    except KeyboardInterrupt:
        pass  # Allow graceful shutdown with Ctrl+C

    serial_twist_pass.destroy()  # Call custom destroy method


if __name__ == "__main__":
    main()
