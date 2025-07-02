import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Header


class Relay(Node):
    def __init__(self):
        super().__init__("relay")

        self.publisher = self.create_publisher(AckermannDriveStamped, '/drive_relay', 10)

        self.subscriber = self.create_subscription(AckermannDriveStamped, '/drive', self.process_data, 10)

    def process_data(self, rec_msg):
        rec_msg.drive.steering_angle *= 3
        rec_msg.drive.speed *= 3
        self.get_logger().info(f"Speed: {rec_msg.drive.speed}, Steering Angle: {rec_msg.drive.steering_angle}")
        self.publisher.publish(rec_msg)
        
def main(args = None):
    rclpy.init(args = args)
    node = Relay()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()