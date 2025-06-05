from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Webcam Publisher Node
        Node(
            package='color_pkg',
            executable='webcam_publisher_node',
            name='webcam_publisher',
            output='screen'
        ),

        # Shape Detector Node
        Node(
            package='color_pkg',
            executable='shape_detector_node',
            name='shape_detector',
            output='screen'
        ),
    ])
