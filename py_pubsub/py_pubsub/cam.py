import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ConeDetector(Node):
    def __init__(self):
        super().__init__('cone_detector')
        self.publisher = self.create_publisher(Int32MultiArray, 'cone_centers_areas', 10)
        self.subscription = self.create_subscription(Image,'/image',self.image_callback,10)
        self.bridge = CvBridge()

    def preprocess(self, img):
        img_blur = cv2.GaussianBlur(img, (5, 5), 1)
        img_canny = cv2.Canny(img_blur, 50, 50)
        kernel = np.ones((3, 3), np.uint8)
        img_dilate = cv2.dilate(img_canny, kernel, iterations=2)
        img_erode = cv2.erode(img_dilate, kernel, iterations=1)

        areas, centers = [], []
        contours, _ = cv2.findContours(img_erode, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        img_cleaned = np.zeros_like(img)

        for cnt in contours:
            if cv2.contourArea(cnt) > 500:
                cv2.drawContours(img_cleaned, [cnt], 0, 255, -1)
                areas.append(cv2.contourArea(cnt))
                M = cv2.moments(cnt)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                else:
                    cX, cY = 0, 0
                centers.append([cX, cY])

        return img_cleaned, areas, centers

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_bound = np.array([100, 25, 25])
        upper_bound = np.array([140, 255, 255])
        mask = cv2.inRange(hsv, lower_bound, upper_bound)
        non_yellow_mask = cv2.bitwise_not(mask)
        cutout = cv2.bitwise_and(frame, frame, mask=mask)
        cutout[non_yellow_mask != 0] = [255, 255, 255]

        cleaned, areas, centers = self.preprocess(cutout)

        msg = Int32MultiArray()
        if areas:
            largest_area = max(areas)
            largest_center = centers[areas.index(largest_area)]
            msg.data = [int(largest_area), *largest_center]
        else:
            msg.data = [0, 0, 0]

        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ConeDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
