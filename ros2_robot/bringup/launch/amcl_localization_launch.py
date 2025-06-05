import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node 

def generate_launch_description():
    # Path to nav2_bringup package (standard installed package)
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    # Absolute path to custom params file
    default_params_file = '/home/illia/ros2_main/src/ros2_robot/bringup/config/nav2_params.yaml'
    # Absolute path to map file (confirmed base name 'map' in /home/pi/ros2_main/)
    default_map_yaml = '/home/illia/ros2_main/map4.yaml'

    # ----- Launch Arguments -----
    # Declare launch arguments that can be passed externally
    #  LaunchConfiguration to access their values
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_yaml_path = LaunchConfiguration('map', default=default_map_yaml)
    params_file = LaunchConfiguration('params_file', default=default_params_file)
    autostart = LaunchConfiguration('autostart', default='true')
    use_respawn = LaunchConfiguration('use_respawn', default='false')

    # ----- Actions -----
    # Declare the launch arguments to the launch system
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=default_map_yaml,
        description='Full path to map file to load')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS 2 parameters file to use for Nav2 nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the Nav2 stack')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn crashed nodes')

    # Include Nav2's standard localization launch file
    # This handles launching map_server, amcl, and the lifecycle manager correctly
    include_nav2_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'localization_launch.py')),
        # Pass the arguments down to the included launch file
        launch_arguments={
            'map': map_yaml_path,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            'use_respawn': use_respawn,
        }.items(),
    )

    # ----- Assemble Launch Description -----
    ld = LaunchDescription()

    # Add declared arguments first
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_respawn_cmd)

    # Add the main action (including Nav2's localization launch)
    ld.add_action(include_nav2_localization)

    return ld