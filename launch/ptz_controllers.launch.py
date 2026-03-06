from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
import os
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    # Get package path
    package_path = get_package_share_directory('ptz_gz_sim')

    # Path to Xacro file
    xacro_path = os.path.join(package_path, 'urdf', 'axis_camera_gazebo.xacro')

    # PTZ description
    ptz_description = {"robot_description": 
                      ParameterValue(Command(['xacro ', xacro_path]), value_type=str)}
   
    # Controller config
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("ptz_gz_sim"), "config", "ptz_controllers.yaml"]
    )

    # Robot State Publisher for PTZ
    ptz_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='ptz_state_publisher',
        output='screen',
        parameters=[ptz_description],
    )

    # Controller manager node
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='both',
        parameters=[
            ptz_description,
            robot_controllers
        ],
    )

    # Spawners
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )
    
    ptz_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ptz_controller", "--param-file", robot_controllers],
        output="screen",
    )

    # Fake axis camera node
    fake_axis_camera = Node(
        package='ptz_gz_sim',
        executable='fake_axis_camera_node',
        name='fake_axis_camera_node',
        output='screen'
    )

    # Delay start of joint_state_broadcaster after ptz_controller
    delay_joint_state_broadcaster_after_ptz_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=ptz_controller_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    return LaunchDescription([
        ptz_state_publisher,
        control_node,
        ptz_controller_spawner,
        delay_joint_state_broadcaster_after_ptz_controller_spawner,
        fake_axis_camera,
    ])