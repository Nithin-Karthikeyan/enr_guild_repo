import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from roboflow import Roboflow
import json

class RoboflowDetectorNode(Node):
    def __init__(self):
        super().__init__('roboflow_detector')
        
        # ROS2 Publishers and Subscribers
        self.detection_publisher = self.create_publisher(String, 'detected_objects', 10)
        self.image_subscriber = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10)
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Timer for webcam mode (if not using image topic)
        self.timer = self.create_timer(0.001, self.timer_callback)
        
        # Initialize Roboflow model
        self.setup_roboflow_model()
        
        # OpenCV webcam (optional - for direct webcam access)
        self.cap = cv2.VideoCapture(0)
        
        # Parameters
        self.confidence_threshold = 50
        self.overlap_threshold = 30
        
        self.get_logger().info('Roboflow Detector Node initialized')

    def setup_roboflow_model(self):
        """Initialize the Roboflow model"""
        try:
            # Replace these with your actual values
            API_KEY = ""
            WORKSPACE = "nithinws"
            PROJECT = "water-bottles-2-yibe7"
            VERSION = 1  # Your model version
            
            rf = Roboflow(api_key=API_KEY)
            project = rf.workspace(WORKSPACE).project(PROJECT)
            self.model = project.version(VERSION).model
            
            self.get_logger().info(f'Roboflow model loaded: {PROJECT} v{VERSION}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to load Roboflow model: {str(e)}')
            self.model = None

    def image_callback(self, msg):
        """Callback for processing images from ROS topic"""
        if self.model is None:
            return
            
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Run detection
            self.process_image(cv_image)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def timer_callback(self):
        """Timer callback for webcam mode"""
        if self.model is None:
            return
            
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to grab webcam frame')
            return
            
        # Process the webcam frame
        self.process_image(frame)

    def process_image(self, image):
        """Process image and run detection"""
        try:
            # Run Roboflow inference
            results = self.model.predict(
                image, 
                confidence=self.confidence_threshold, 
                overlap=self.overlap_threshold
            ).json()
            print(type(results))
            
            # Extract detection information
            detections = results.get('predictions', [])
            
            # Create detection message
            detection_data = {
                'timestamp': str(self.get_clock().now().to_msg()),
                'num_detections': len(detections),
                'detections': []
            }
            print(detection_data['timestamp'])
            
            # Process each detection
            for detection in detections:
                detection_info = {
                    'class': detection['class'],
                    'confidence': detection['confidence'],
                    'x': detection['x'],
                    'y': detection['y'],
                    'width': detection['width'],
                    'height': detection['height']
                }
                
                # Calculate distance from image center
                img_center_x = image.shape[1] // 2
                img_center_y = image.shape[0] // 2
                distance_from_center = ((detection['x'] - img_center_x)**2 + 
                                      (detection['y'] - img_center_y)**2)**0.5
                detection_info['distance_from_center'] = distance_from_center
                
                detection_data['detections'].append(detection_info)
            # Publish detection results
            msg = String()
            msg.data = json.dumps(detection_data)
            self.detection_publisher.publish(msg)
            
            # Log detection results
            if detections:
                classes = [d['class'] for d in detections]
                self.get_logger().info(f'Detected: {", ".join(classes)}')
            
            # Display image with annotations (optional)
            self.display_results(image, detections)
            
        except Exception as e:
            self.get_logger().error(f'Error in detection: {str(e)}')

    def display_results(self, image, detections):
        """Display image with bounding boxes and labels"""
        display_image = image.copy()
        
        for detection in detections:
            # Calculate bounding box coordinates
            x = int(detection['x'])
            y = int(detection['y'])
            w = int(detection['width'])
            h = int(detection['height'])
            
            x1 = int(x - w/2)
            y1 = int(y - h/2)
            x2 = int(x + w/2)
            y2 = int(y + h/2)
            
            # Draw bounding box
            cv2.rectangle(display_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # Draw label
            label = f"{detection['class']}: {detection['confidence']:.2f}"
            cv2.putText(display_image, label, (x1, y1-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Draw center point
            cv2.circle(display_image, (x, y), 5, (0, 0, 255), -1)
        
        # Show image
        cv2.imshow('Roboflow Detection', display_image)
        cv2.waitKey(1)

    def destroy_node(self):
        """Clean up resources"""
        if hasattr(self, 'cap'):
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RoboflowDetectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

