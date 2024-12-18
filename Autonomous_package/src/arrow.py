#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import cv2
import numpy as np
import math

class ArrowDirectionPublisher(Node):
    def __init__(self):
        super().__init__('arrow_direction_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)  # publishing to 'cmd_vel' topic
        self.cap = cv2.VideoCapture(0)
        self.right = cv2.imread("/home/yagna/Desktop/Code/Rover/Images/right.jpg", cv2.IMREAD_GRAYSCALE)
        self.left = cv2.imread("/home/yagna/Desktop/Code/Rover/Images/left.jpg", cv2.IMREAD_GRAYSCALE)
        self.threshold = 0.8
        self.timer = self.create_timer(0.1, self.publish_direction_and_angle)  # Publishes at 10Hz

    def preprocess(self, img):
        img_canny = cv2.Canny(img, 50, 50)
        kernel = np.ones((3, 3))
        img_dilate = cv2.dilate(img_canny, kernel, iterations=2)
        img_erode = cv2.erode(img_dilate, kernel, iterations=1)
        return img_erode

    def find_tip(self, points, convex_hull):
        length = len(points)
        indices = np.setdiff1d(range(length), convex_hull)

        for i in range(2):
            j = indices[i] + 2
            if j > length - 1:
                j = length - j
            if np.all(points[j] == points[indices[i - 1] - 2]):
                return tuple(points[j])

    def convert_to_binary(self, frame):
        original_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred_image = cv2.GaussianBlur(original_image, (5, 5), 0)
        return blurred_image

    def direction(self, approx, arrow_tip):
        if len(approx) >= 7:
            l = r = 0
            for a in approx:
                if a[0, 0] > arrow_tip[0]:
                    l += 1
                if a[0, 0] < arrow_tip[0]:
                    r += 1
            if l > 4:
                return -1  # Left
            if r > 4:
                return 1  # Right
            else:
                return 0  # No significant direction
        else:
            return 0

    def multi_scale_template_matching(self, image, template):
        max_value = -1
        maxi_loc = -1
        max_scale = -1
        for scale in np.linspace(0.08, 0.5, 20):
            scaled_template = cv2.resize(template, None, fx=scale, fy=scale)
            result = cv2.matchTemplate(image, scaled_template, cv2.TM_CCOEFF_NORMED)
            _, max_val, _, max_loc = cv2.minMaxLoc(result)
            if max_val >= self.threshold and max_val > max_value:
                max_value = max_val
                maxi_loc = max_loc
                max_scale = scale
        return maxi_loc, max_scale

    def publish_direction_and_angle(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to capture frame from camera.')
            return
        
        frame1 = self.convert_to_binary(frame)
        contours, _ = cv2.findContours(self.preprocess(frame1), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        twist = Twist()

        for cnt in contours:
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
            hull = cv2.convexHull(approx, returnPoints=False)
            sides = len(hull)

            if 6 > sides > 4 and sides + 2 == len(approx) and len(approx) > 6:
                arrow_tip = self.find_tip(approx[:, 0, :], hull.squeeze())
                if arrow_tip and len(approx) >= 7:
                    dir = self.direction(approx, arrow_tip)
                    if dir != 0:
                        # twist.linear.x = 0.5  # Move forward
                        twist.angular.z = dir * 0.5  # Adjust rotation based on direction
                        self.publisher_.publish(twist)
                        print(f'Published Direction: {dir}')
        
        match_loc, match_scale = self.multi_scale_template_matching(frame1, self.right)
        if match_loc != -1:
            w = int(self.right.shape[1] * match_scale)
            h = int(self.right.shape[0] * match_scale)
            cv2.rectangle(frame, match_loc, (match_loc[0] + w, match_loc[1] + h), (0, 255, 0), 2)
            degree = math.degrees(math.atan((match_loc[0] - 320.0 + 0.5 * w) / match_loc[1]))
            print(f'Right detected, angle: {degree}')
        
        match_loc, match_scale = self.multi_scale_template_matching(frame1, self.left)
        if match_loc != -1:
            w = int(self.left.shape[1] * match_scale)
            h = int(self.left.shape[0] * match_scale)
            cv2.rectangle(frame, match_loc, (match_loc[0] + w, match_loc[1] + h), (255, 0, 0), 2)

        cv2.imshow("Image", frame)
        cv2.imshow("Processed Image", frame1)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.cap.release()
            cv2.destroyAllWindows()
            self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    arrow_direction_publisher = ArrowDirectionPublisher()
    rclpy.spin(arrow_direction_publisher)
    arrow_direction_publisher.cap.release()
    cv2.destroyAllWindows()
    arrow_direction_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()