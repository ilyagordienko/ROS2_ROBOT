import rclpy 
from rclpy.node import Node 
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge 
import cv2 
import numpy as np 

class ColorDetectorNode(Node):
    def __init__(self):
        super().__init__('color_detector_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10)
        self.subscription
        self.bridge = CvBridge()

        self.color_ranges = {
            'Blue':    {'lower': np.array([100, 150, 50]),   'upper': np.array([140, 255, 255]), 'color': (255, 0, 0)},
            'Green':   {'lower': np.array([35, 100, 50]),    'upper': np.array([85, 255, 255]),  'color': (0, 255, 0)},
            'Black':   {'lower': np.array([0, 0, 0]),        'upper': np.array([180, 120, 80]),  'color': (0, 0, 0)},
            'Yellow':  {'lower': np.array([10, 50, 70]),     'upper': np.array([50, 255, 255]),  'color': (0, 255, 255)},
            'Pink':    {'lower': np.array([140, 50, 100]),   'upper': np.array([170, 255, 255]), 'color': (203, 192, 255)},
            'Red': {
                'lower1': np.array([0, 120, 70]), 
                'upper1': np.array([10, 255, 255]),
                'lower2': np.array([170, 120, 70]),
                'upper2': np.array([180, 255, 255]),
                'color': (0, 0, 255)
            }
        }

    def listener_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        for color_name, color_range in self.color_ranges.items():
            if color_name == 'Red':
                mask1 = cv2.inRange(hsv, color_range['lower1'], color_range['upper1'])
                mask2 = cv2.inRange(hsv, color_range['lower2'], color_range['upper2'])
                mask = cv2.bitwise_or(mask1, mask2)
            else:
                mask = cv2.inRange(hsv, color_range['lower'], color_range['upper'])

            result = cv2.bitwise_and(frame, frame, mask=mask)
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 500:
                    x, y, w, h = cv2.boundingRect(contour)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), color_range['color'], 2)

                    roi_mask = mask[y:y + h, x:x + w]
                    confidence = np.sum(roi_mask > 0) / (w * h)
                    label = f"{color_name}: {confidence:.2f}"
                    cv2.putText(frame, label, (x, y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_range['color'], 2)

        cv2.imshow("Detected Colors", frame)
        cv2.waitKey(1)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ColorDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
