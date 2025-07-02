import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Circle(Node):
    def __init__(self):
        super().__init__("Circle")
        self.vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.publish)
        self.twist = Twist()

    def publish(self):
        self.twist.linear.x = 1.0
        self.twist.angular.z = 2.0
        self.vel_pub.publish(self.twist)
        self.get_logger().info("Spinning around")

def main(args=None):
    rclpy.init(args=args)
    node = Circle()
    rclpy.spin(node=node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()