#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', 
        default_value='true',
        description='Use simulation time'
    )

    # Path configurations  
    ptz_gz_sim_path = get_package_share_directory('ptz_gz_sim')
    urdf_file = os.path.join(ptz_gz_sim_path, 'urdf', 'axis_camera_gazebo.xacro')
    controllers_file = os.path.join(ptz_gz_sim_path, 'config', 'ptz_controllers.yaml')

    # Robot State Publisher - PTZ URDF
    robot_state_publisher_ptz = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_ptz',
        output='screen',
        parameters=[
            {'robot_description': open(urdf_file).read()},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static')
        ]
    )

    # Entity spawner - Spawn PTZ in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_entity_ptz',
        output='screen',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'axis_camera',
            '-x', '0.0',
            '-y', '0.0', 
            '-z', '1.0'
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Controller Manager for PTZ
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='ptz_controller_manager',
        output='screen',
        parameters=[
            controllers_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    # Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='joint_state_broadcaster_spawner',
        output='screen',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Position Controller Spawner (ONLY position controller for integration version)
    position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='position_controller_spawner',
        output='screen',
        arguments=['position_controller', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Fake Axis Camera Integration Node
    fake_axis_camera_int_node = Node(
        package='ptz_gz_sim',
        executable='fake_axis_camera_int_node',  # This will be the integration version executable
        name='fake_axis_camera',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Gazebo-ROS bridge for camera (PTZ Camera)
    gz_ros2_bridge_ptz = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_ros2_bridge_ptz',
        output='screen',
        arguments=[
            '/axis_camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/axis_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '--ros-args', 
            '-p', 'use_sim_time:=true'
        ],
        remappings=[
            ('/axis_camera/image_raw', '/image_raw'),
            ('/axis_camera/camera_info', '/camera_info')
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        robot_state_publisher_ptz,
        TimerAction(period=2.0, actions=[spawn_entity]),
        TimerAction(period=3.0, actions=[controller_manager]),
        TimerAction(period=5.0, actions=[joint_state_broadcaster_spawner]),
        TimerAction(period=6.0, actions=[position_controller_spawner]),
        TimerAction(period=7.0, actions=[fake_axis_camera_int_node]),
        TimerAction(period=8.0, actions=[gz_ros2_bridge_ptz]),
    ])