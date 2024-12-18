#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge

class ObjectDetectionSubscriber(Node):
    def __init__(self):
        super().__init__('object_detection_subscriber')
        self.subscription = self.create_subscription(
            Image, 
            'camera_feed', 
            self.listener_callback, 
            10
        )
        self.bridge = CvBridge()

    def detect_objects(self, frame):
        # Convert BGR to HSV
        hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Define color range (e.g., blue)
        lower_range = np.array([100, 150, 50])
        upper_range = np.array([140, 255, 255])
        
        # Create a mask
        mask = cv2.inRange(hsv_image, lower_range, upper_range)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        # Draw contours on the frame
        for contour in contours:
            if cv2.contourArea(contour) > 500:  # Filter small objects
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
        
        return frame, mask

    def listener_callback(self, msg):
        # Convert ROS Image message to OpenCV format
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Perform object detection
        detected_frame, mask = self.detect_objects(frame)
        
        # Display the frames
        cv2.imshow('Detected Objects', detected_frame)
        cv2.imshow('Mask', mask)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
