import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import mediapipe as mp

class FingerCounterNode(Node):
    def __init__(self):
        super().__init__('finger_counter_node')
        self.publisher_ = self.create_publisher(Int32, 'finger_count', 10)
        self.image_pub = self.create_publisher(Image, 'annotated_image', 10)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(static_image_mode=False, max_num_hands=1)
        self.mp_draw = mp.solutions.drawing_utils

    def count_fingers(self, hand_landmarks):
        # List of tip landmarks for thumb, index, middle, ring, pinky
        tips = [4, 8, 12, 16, 20]
        fingers = []

        # Thumb (special case: compare x for left/right hand)
        if hand_landmarks.landmark[tips[0]].x < hand_landmarks.landmark[tips[0] - 1].x:
            fingers.append(1)
        else:
            fingers.append(0)

        # Other four fingers
        for i in range(1, 5):
            if hand_landmarks.landmark[tips[i]].y < hand_landmarks.landmark[tips[i] - 2].y:
                fingers.append(1)
            else:
                fingers.append(0)
        return sum(fingers)

    def run(self):
        while rclpy.ok():
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().error("Failed to grab frame")
                continue

            img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = self.hands.process(img_rgb)
            finger_count = 0

            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    finger_count = self.count_fingers(hand_landmarks)
                    self.mp_draw.draw_landmarks(
                        frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                    # Draw finger count on image
                    cv2.putText(frame, f'Fingers: {finger_count}', (10, 50),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            # Publish finger count
            msg = Int32()
            msg.data = finger_count
            self.publisher_.publish(msg)

            # Publish annotated image
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.image_pub.publish(img_msg)

            # Optional: show image (for debugging)
            cv2.imshow("Hand Detection", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = FingerCounterNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
