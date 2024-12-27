import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import cv2
import numpy as np
import math

# Constants
MATCH_THRESHOLD = 0.8


# Preprocessing: Edge detection and grayscale conversion
def edge_detection(image):
    edges = cv2.Canny(image, 50, 150)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel, iterations=2)
    return edges

def to_grayscale_and_blur(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (7, 7), 0)
    return blurred

# Contour detection
def detect_contours(image):

    processed = edge_detection(to_grayscale_and_blur(image))
    contours, _ = cv2.findContours(processed, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    return contours

# Helper functions for arrow detection
def identify_arrow_tip(points, hull_indices):
    for i in range(len(points)):
        if i not in hull_indices:
            next_index = (i + 2) % len(points)
            prev_index = (i - 2 + len(points)) % len(points)
            
            # Check if the points are approximately equal
            distance = np.linalg.norm(points[next_index] - points[prev_index])
            if distance < 1e-3:  # Threshold for considering points equal
                return points[next_index]
    return (-1, -1)

def determine_direction(approx, tip):
    left_points = sum(1 for pt in approx if pt[0][0] < tip[0])
    right_points = sum(1 for pt in approx if pt[0][0] > tip[0])

    if left_points > right_points and left_points > 4:
        return "Left"
    if right_points > left_points and right_points > 4:
        return "Right"
    return "None"

# Template matching
def match_and_annotate(frame, template_img, color, label):
    
    gray_frame = to_grayscale_and_blur(frame)
    best_value = -1
    best_location = (-1, -1)
    best_scale = -1

    for scale in np.arange(0.1, 0.5, 0.027):
        resized_template = cv2.resize(template_img, None, fx=scale, fy=scale)
        result = cv2.matchTemplate(gray_frame, resized_template, cv2.TM_CCOEFF_NORMED)
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)

        if max_val > best_value and max_val > MATCH_THRESHOLD:
            best_value = max_val
            best_location = max_loc
            best_scale = scale

    if best_location != (-1, -1):
        w = int(template_img.shape[1] * best_scale)
        h = int(template_img.shape[0] * best_scale)
        top_left = best_location
        bottom_right = (top_left[0] + w, top_left[1] + h)
        cv2.rectangle(frame, top_left, bottom_right, color, 2)
        
        # Calculate the center of the detected arrow
        arrow_center_x = top_left[0] + w // 2

        # Calculate the deviation from the center of the screen to the center of the detected arrow on the x-axis
        frame_center_x = frame.shape[1] // 2
        deviation_x = arrow_center_x - frame_center_x

        # Return the direction and deviation
        return label, deviation_x

    return None, None

# Process each frame to detect arrows
def process_frame(frame, right_arrow, left_arrow):
    contours = detect_contours(frame)
    for contour in contours:
        approx = cv2.approxPolyDP(contour, 0.02 * cv2.arcLength(contour, True), True)
        hull = cv2.convexHull(approx, returnPoints=False)

        if 4 < len(hull) < 6 and len(hull) + 2 == len(approx) and len(approx) > 6:
            tip = identify_arrow_tip(approx, hull)
            if tip != (-1, -1):
                direction = determine_direction(approx, tip)
                if direction != "None":
                    cv2.drawContours(frame, [contour], -1, (0, 255, 0), 3)
                    cv2.circle(frame, tip, 3, (0, 0, 255), -1)
                    print(f"Arrow Direction: {direction}")

    right_direction, right_deviation = match_and_annotate(frame, right_arrow, (0, 255, 0), "-1")
    left_direction, left_deviation = match_and_annotate(frame, left_arrow, (255, 0, 0), "1")

    if right_direction:
        return right_direction, right_deviation
    elif left_direction:
        return left_direction, left_deviation
    else:
        return None, None

class ArrowDetector(Node):

    def __init__(self):
        super().__init__('arrow_detector')
        self.publisher_ = self.create_publisher(Twist, 'arrow_info', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        
        self.right_arrow = cv2.imread("src/py_pubsub/py_pubsub/right.jpeg", cv2.IMREAD_GRAYSCALE)
        self.left_arrow = cv2.imread("src/py_pubsub/py_pubsub/left.jpeg", cv2.IMREAD_GRAYSCALE)
        
        if self.right_arrow is None or self.left_arrow is None:
            self.get_logger().error("Error loading template images")
            return


        self.cap = cv2.VideoCapture(2)
        if not self.cap.isOpened():
            self.get_logger().error("Cannot open webcam")
            return

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret or frame is None:
            self.get_logger().error("Error capturing frame")
            return

        direction, deviation = process_frame(frame, self.right_arrow, self.left_arrow)
        if direction:
            msg = Twist()
            msg.linear.z =  float(direction)
            msg.linear.y =  float(deviation)
            self.publisher_.publish(msg)
            self.get_logger().info(f"Published: direction={direction}, deviation={deviation}")


        cv2.imshow("Video Feed", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.cap.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    arrow_detector = ArrowDetector()
    rclpy.spin(arrow_detector)
    arrow_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
