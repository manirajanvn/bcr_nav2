#!/usr/bin/env python3

from os.path import join
import xacro
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def get_xacro_to_doc(xacro_file_path, mappings):
    doc = xacro.parse(open(xacro_file_path))
    xacro.process_doc(doc, mappings=mappings)
    return doc

def generate_launch_description():
    # Retrieve launch configuration arguments
    package_name = 'bcr_bot'
    pkg_share = FindPackageShare(package=package_name).find(package_name)

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    position_x = LaunchConfiguration("position_x")
    position_y = LaunchConfiguration("position_y")
    orientation_yaw = LaunchConfiguration("orientation_yaw")
    camera_enabled = LaunchConfiguration("camera_enabled", default=True)
    two_d_lidar_enabled = LaunchConfiguration("two_d_lidar_enabled", default=True)
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    slam = LaunchConfiguration('slam')
    # Path to the Xacro file
    bcr_pkg = get_package_share_directory('bcr_bot')
    xacro_path = join(bcr_pkg, 'urdf', 'bcr_bot.xacro')
    nav2_params_path = 'config/nav2_params.yaml'

    map_file_path = 'map/map.yaml'
    nav2_params_path = 'config/nav2_params.yaml'
    robot_localization_file = 'config/ekf.yaml'
    robot_localization_file_path = os.path.join(pkg_share, robot_localization_file) 
    nav2_dir = FindPackageShare(package='nav2_bringup').find('nav2_bringup') 

    #doc = get_xacro_to_doc(xacro_path, {"wheel_odom_topic": "odom", "sim_gazebo": "true", "two_d_lidar_enabled": "true", "camera_enabled": "true"})

    
    map_yaml_file = LaunchConfiguration('map')
    static_map_path = os.path.join(pkg_share, map_file_path)
    nav2_params_path = os.path.join(pkg_share, nav2_params_path)
    nav2_launch_dir = os.path.join(nav2_dir, 'launch') 

    remappings = [('/tf', 'tf'),
                ('/tf_static', 'tf_static')]

    declare_slam_cmd = DeclareLaunchArgument(
    name='slam',
    default_value='False',
    description='Whether to run SLAM')
    
    declare_map_yaml_cmd = DeclareLaunchArgument(
    name='map',
    default_value=static_map_path,
    description='Full path to map file to load')

    declare_params_file_cmd = DeclareLaunchArgument(
    name='params_file',
    default_value=nav2_params_path,
    description='Full path to the ROS2 parameters file to use for all launched nodes')
  
    declare_autostart_cmd = DeclareLaunchArgument(
    name='autostart', 
    default_value='true',
    description='Automatically startup the nav2 stack')

    # Launch the robot_state_publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{"use_sim_time": use_sim_time},
                    {'robot_description': Command( \
                    ['xacro ', join(xacro_path),
                    ' camera_enabled:=', camera_enabled,
                    ' two_d_lidar_enabled:=', two_d_lidar_enabled,
                    ' sim_gazebo:=', "true"
                    ])}],
        remappings=remappings)

    # Launch the spawn_entity node to spawn the robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-topic', "/robot_description",
            '-entity', "bcr_bot",
            '-x', position_x,
            '-y', position_y,
            '-Y', orientation_yaw
        ]
    )

    # Include the Gazebo launch file
    gazebo_share = get_package_share_directory("gazebo_ros")
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(gazebo_share, "launch", "gazebo.launch.py"))
    )
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(bcr_pkg, "launch", "rviz.launch.py"))
    )

    start_robot_localization_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[robot_localization_file_path, 
        {'use_sim_time': use_sim_time}])

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    start_ros2_navigation_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'bringup_launch.py')),
    launch_arguments = {
                        'slam': slam,
                        'map': map_yaml_file,
                        'use_sim_time': use_sim_time,
                        'params_file': params_file,
                        'autostart': autostart}.items())

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('world', default_value=[FindPackageShare('bcr_bot'), '/worlds/small_warehouse.sdf']),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('verbose', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument("camera_enabled", default_value = camera_enabled),
        DeclareLaunchArgument("two_d_lidar_enabled", default_value = two_d_lidar_enabled),
        DeclareLaunchArgument("position_x", default_value="0.0"),
        DeclareLaunchArgument("position_y", default_value="0.0"),
        DeclareLaunchArgument("orientation_yaw", default_value="0.0"),   
        # DeclareLaunchArgument('robot_description', default_value=doc.toxml()),
        gazebo,
        robot_state_publisher,
        spawn_entity,
        declare_map_yaml_cmd,
        declare_params_file_cmd,
        declare_autostart_cmd,
        declare_slam_cmd,
        start_robot_localization_cmd,
        joint_state_publisher_node,
        start_ros2_navigation_cmd,
        rviz,      
    ])
