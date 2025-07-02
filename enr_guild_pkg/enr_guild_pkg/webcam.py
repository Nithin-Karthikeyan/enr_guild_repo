import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        self.publisher_ = self.create_publisher(String, 'detected_objects', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.model = YOLO('yolov8n.pt')  # Download this model if not present
        self.cap = cv2.VideoCapture(0)   # Use your webcam

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to grab frame')
            return

        results = self.model(frame)
        labels = [self.model.names[int(cls)] for cls in results[0].boxes.cls]
        msg = String()
        msg.data = ', '.join(labels)
        self.publisher_.publish(msg)

        # Optionally, show frame with bounding boxes
        annotated = results[0].plot()
        cv2.imshow('YOLO Detection', annotated)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    rclpy.spin(node)
    node.cap.release()
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
