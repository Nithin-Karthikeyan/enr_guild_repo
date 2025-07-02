import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Header


class Talker(Node):
    def __init__(self):
        super().__init__("talker")
        self.publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        # self.subscriber = self.create_subscription(AckermannDriveStamped, '/drive', self.callback, 10)

        self.timer = self.create_timer(0, self.pub_msg)
    
    def pub_msg(self):
        # Get the ackermann values (v, d) 
        v = 1.0# m/s
        d = 0.4 # radians
        
        # Publish to topic
        msg = AckermannDriveStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base"
        msg.drive.steering_angle = d # rad
        msg.drive.speed = v # m/s

        self.get_logger().info(f"Speed: {msg.drive.speed}, Steering Angle: {msg.drive.steering_angle}")

        self.publisher.publish(msg)

def main(args = None):
    rclpy.init(args = args)
    node = Talker()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()