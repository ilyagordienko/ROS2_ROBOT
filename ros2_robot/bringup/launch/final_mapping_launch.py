import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    #  absolute paths for  ros2_robot launch files.
    mapping_launch = '/home/illia/ros2_main/src/ros2_robot/bringup/launch/online_async_launch.py'
    diffbot_launch = '/home/illia/ros2_main/src/ros2_robot/bringup/launch/start_robot.launch.py'
    
    # For rplidar_ros  get_package_share_directory
    rplidar_share = get_package_share_directory('rplidar_ros')
    rplidar_launch = os.path.join(rplidar_share, 'launch', 'rplidar_a2m8_launch.py')
    
    # Launch  twist_converter node with a remapping.
    twist_converter_node = Node(
        package='your_twist_converter',
        executable='twist_converter',
        output='screen',
        remappings=[('cmd_vel_stamped', '/diffbot_base_controller/cmd_vel')]
    )
    
    ld = LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mapping_launch)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(diffbot_launch)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rplidar_launch)
        ),
        twist_converter_node,
    ])
    
    return ld