#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(Image, 'camera_feed', 10)
        self.timer = self.create_timer(0.1, self.publish_frame)  # 10 Hz
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(1)  # Open the default webcam
        if not self.cap.isOpened():
            self.get_logger().error("Could not open webcam")

    def publish_frame(self):
        ret, frame = self.cap.read()
        if ret:
            # Publish the frame as an Image message
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
