import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import select

class TeleopTurtle(Node):
    def __init__(self):
        super().__init__('teleop_turtle')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.speed = 2.0
        self.turn = 1.0

    def get_key(self):
        # Capture keyboard input without blocking
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist:
                key = sys.stdin.read(1)
            else:
                key = ''
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def run(self):
        print("Use arrow keys or WASD to move the turtle. Press 'q' to quit. Press 'o'/'p' to increase/decrease speed .")
        while True:
            key = self.get_key()
            twist = Twist()

            if key == 'w' or key == '\x1b[A':  # Up arrow or 'w'
                twist.linear.x = self.speed
            elif key == 's' or key == '\x1b[B':  # Down arrow or 's'
                twist.linear.x = -self.speed
            elif key == 'a' or key == '\x1b[D':  # Left arrow or 'a'
                twist.angular.z = self.turn
            elif key == 'd' or key == '\x1b[C':  # Right arrow or 'd'
                twist.angular.z = -self.turn
            elif key == 'o':
                self.speed += 0.5
            elif key == 'p':
                self.speed -= 0.5
            elif key == 'q':
                print("Quitting...")
                break
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0

            self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    teleop = TeleopTurtle()
    try:
        teleop.run()
    except KeyboardInterrupt:
        pass
    teleop.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
