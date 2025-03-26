import select  # non blocking keyboard input
import sys  # monitor system state
import termios  # for handling tele_op terminal
import threading  # create threads
import tty  # for handling teleop_terminal

import rclpy

# import gemoetry twist message
from geometry_msgs.msg import Twist
from rclpy.node import Node

stop_threads = False  # define a boolean to kill keyboard listener thread


# initialize twist publisher node
class TwistPublisher(Node):
    def __init__(self):
        super().__init__("twist_pub")
        self.publisher_ = self.create_publisher(Twist, "desired_twist", 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # create a key_state dictionary
        self.key_state = {"w": False, "s": False, "a": False, "d": False}

        # define linear and angular speed paramters
        self.declare_parameter("linear_speed", 0.5)
        self.declare_parameter("angular_speed", 1.0)
        self.linear_speed = self.get_parameter("linear_speed").value
        self.angular_speed = self.get_parameter("angular_speed").value

        # start a seperate thread to listen for keyboard input
        self.keyboard_thread1 = threading.Thread(target=self.keyboard_listener)
        self.keyboard_thread1.start()
        # Need to find a way to shut down the thread properly

    # create timer callback: every 0.5 seconds, the function should check for
    # keyboard input and calculate the twist message to send based on what keys
    # are pressed
    def keyboard_listener(self):
        # save current terminal settings
        old_terminal_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())  # define terminal as a stream of input
            while rclpy.ok():
                global stop_threads
                if stop_threads:
                    break
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key_pressed = sys.stdin.read(1)

                    if key_pressed in self.key_state:
                        # read the system input, and assign states to the dictionary
                        self.key_state[key_pressed] = True  # key press detection
                    else:
                        # resets keystates if the key that was pressed is not in dictionary
                        self.reset_key_states()

                else:
                    # resets keystates if no key was pressed in 0.1 seconds
                    self.reset_key_states()

        except KeyboardInterrupt:
            sys.exit()

        finally:
            # restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_terminal_settings)

    def timer_callback(self):
        msg = Twist()

        # set linear and angular velocity based on keyboard inputs
        if self.key_state["w"]:
            msg.linear.x = self.linear_speed
        elif self.key_state["s"]:
            msg.linear.x = -self.linear_speed

        if self.key_state["d"]:
            msg.angular.z = self.angular_speed
        elif self.key_state["a"]:
            msg.angular.z = -self.angular_speed

        # publish the twist message
        self.publisher_.publish(msg)
        self.get_logger().info("Linear velocity = %f" % msg.linear.x)
        self.get_logger().info("Angular velocity = %f" % msg.angular.z)

    def reset_key_states(self):
        for key in self.key_state:
            self.key_state[key] = False


def main(args=None):
    global stop_threads
    stop_threads = False
    rclpy.init(args=args)

    twist_publisher = TwistPublisher()  # define a twist publisher node
    try:
        rclpy.spin(twist_publisher)
    except KeyboardInterrupt:
        stop_threads = True
        sys.exit()

    twist_publisher.destroy_node()
    rclpy.shutdown()
