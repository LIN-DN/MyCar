import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import sys, select, termios, tty

# Function to read keyboard input
def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

# ROS 2 Node class
class KeyboardJoyPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_joy_publisher')
        self.publisher_ = self.create_publisher(Joy, 'joy', 10)
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.axes = [0.0, 0.0, 0.0, 0.0]  # Assuming 4 axes
        msg.buttons = [0] * 10  # Assuming 10 buttons

        key = getKey()
        if key == 'w':
            msg.axes[1] = 1.0
        elif key == 's':
            msg.axes[1] = -1.0
        elif key == 'a':
            msg.axes[0] = -1.0
        elif key == 'd':
            msg.axes[0] = 1.0
        elif key == '\x0a' or key == '\r' :
            msg.buttons[0] = 1  # Button a pressed
        elif key == '\x7f':
            msg.buttons[1] = 1  # Button b pressed
        elif key == 'r':
            msg.buttons[5] = 1  # Button R1 pressed
        elif key == 'l':
            msg.buttons[4] = 1  # Button L1 pressed
        elif key == 'q':  # Press 'q' to quit
            sys.exit()

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    global settings
    settings = termios.tcgetattr(sys.stdin)

    keyboard_joy_publisher = KeyboardJoyPublisher()

    try:
        rclpy.spin(keyboard_joy_publisher)
    except KeyboardInterrupt:
        # pass
        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

        keyboard_joy_publisher.destroy_node()
        rclpy.shutdown()
        sys.exit()

if __name__ == '__main__':
    main()
