from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Webcam Publisher Node
        Node(
            package='color_pkg',  #
            executable='webcam_publisher_node',  # The executable of  webcam publisher
            name='webcam_publisher',
            output='screen'
        ),
        
        # Color Detection Node
        Node(
            package='color_pkg',  #
            executable='color_detector_node', 
            name='color_detector',
            output='screen'
        ),
    ])
