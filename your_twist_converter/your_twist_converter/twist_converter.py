#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class TwistConverter(Node):
    def __init__(self):
        super().__init__('twist_converter')
        # Subscribe to plain Twist messages (e.g., from teleop_twist_keyboard)
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10
        )
        # Publish TwistStamped messages (for the controller)
        self.publisher = self.create_publisher(
            TwistStamped,
            'cmd_vel_stamped',
            10
        )
        self.get_logger().info('Twist Converter node has been started.')

    def listener_callback(self, twist_msg):
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = 'base_link'  # Adjust if needed

        # Apply a 5x coefficient to linear and angular speeds
        twist_stamped.twist.linear.x = - twist_msg.linear.x * 50.0
        twist_stamped.twist.linear.y = twist_msg.linear.y * 50.0
        twist_stamped.twist.linear.z = - twist_msg.linear.z * 135.0
        twist_stamped.twist.angular.x = twist_msg.angular.x * 50.0
        twist_stamped.twist.angular.y = twist_msg.angular.y * 50.0
        twist_stamped.twist.angular.z = - twist_msg.angular.z * 135.0

        self.publisher.publish(twist_stamped)
        self.get_logger().info('Converted Twist to TwistStamped with 30x scaling')

def main(args=None):
    rclpy.init(args=args)
    node = TwistConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
