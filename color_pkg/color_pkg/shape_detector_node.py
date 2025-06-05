import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge 
import cv2
import numpy as np 

class ShapeDetectorNode(Node):
    def __init__(self):
        super().__init__('shape_detector_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10)
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Adaptive thresholding for better shape detection in variable lighting
        thresh = cv2.adaptiveThreshold(
            gray, 255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV,
            11, 3
        )

        # Morphological closing to fill small holes and smooth edges
        kernel = np.ones((5, 5), np.uint8)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 1000:
                continue

            peri = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, 0.04 * peri, True)
            x, y, w, h = cv2.boundingRect(approx)

            shape = None
            color = (255, 255, 255)

            if len(approx) == 3:
                shape = "Triangle"
                color = (0, 0, 255)  # Red
            elif len(approx) == 4:
                aspect_ratio = float(w) / h
                if 0.9 < aspect_ratio < 1.1:
                    shape = "Square"
                    color = (255, 0, 0)  # Blue
            elif len(approx) > 6:
                shape = "Circle"
                color = (0, 255, 0)  # Green

            if shape:
                cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
                cv2.putText(frame, shape, (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        cv2.imshow("Filled Shape Detection", frame)
        cv2.waitKey(1)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ShapeDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
