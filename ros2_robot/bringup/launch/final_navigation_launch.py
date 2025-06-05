import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Define the default map path 
    default_map_path = '/home/illia/ros2_main/map4.yaml'

    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value=default_map_path,
        description='Full path to map file to load')
    #  LaunchConfiguration to access the argument's value later
    map_yaml_path = LaunchConfiguration('map')

    #  absolute paths for ros2_robot launch files.
    navigation_launch = '/home/illia/ros2_main/src/ros2_robot/bringup/launch/navigation_launch.py'
    diffbot_launch = '/home/illia/ros2_main/src/ros2_robot/bringup/launch/start_robot.launch.py'

    # localization_launch = '/home/illia/ros2_main/src/ros2_robot/bringup/launch/localization_launch.py' # OLD
    localization_launch = '/home/illia/ros2_main/src/ros2_robot/bringup/launch/amcl_localization_launch.py' # NEW 

    # For rplidar_ros get_package_share_directory 
    rplidar_share = get_package_share_directory('rplidar_ros')
    rplidar_launch = os.path.join(rplidar_share, 'launch', 'rplidar_a2m8_launch.py') # Ensure this is the correct launch file name

    # Define launch descriptions for each component
    diffbot_launch_desc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(diffbot_launch)
    )

    # Launch  twist_converter node with a remapping.
    twist_converter_node = Node(
        package='your_twist_converter',
        executable='twist_converter',
        output='screen',
        remappings=[('cmd_vel_stamped', '/diffbot_base_controller/cmd_vel')]
    )

    rplidar_launch_desc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rplidar_launch)
    )

    localization_launch_desc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_launch),
        #  launch_arguments to pass the map path down
        launch_arguments={'map': map_yaml_path}.items() # added later
    )

    navigation_launch_desc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch)
    )

    #  TimerAction to sequence the launch files
    twist_converter_timer = TimerAction(
        period=10.0,
        actions=[twist_converter_node]
    )

    rplidar_timer = TimerAction(
        period=15.0,
        actions=[rplidar_launch_desc]
    )

    localization_timer = TimerAction(
        period=20.0,
        actions=[localization_launch_desc] # This now includes the modified version
    )

    navigation_timer = TimerAction(
        period=35.0,
         actions=[navigation_launch_desc]
    )

    # Return the launch description with timed execution
    ld = LaunchDescription([
        declare_map_cmd,

        diffbot_launch_desc,
        twist_converter_timer,
        rplidar_timer,
        localization_timer,
        navigation_timer
    ])

    return ld