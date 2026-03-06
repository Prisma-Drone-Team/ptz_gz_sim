from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
import os


def generate_launch_description():
    # Get package path
    package_path = get_package_share_directory('ptz_gz_sim')

    # Launch arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        )
    )

    # Configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')

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
        parameters=[ptz_description, {'use_sim_time': use_sim_time}],
    )

    # Controller manager node
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[
            ptz_description,
            robot_controllers,
            {'use_sim_time': use_sim_time}
        ],
    )

    # Joint state broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
        parameters=[{'use_sim_time': use_sim_time}],
    )
    
    # Position controller spawner (active by default)
    position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller"],
        output="screen",
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Velocity controller spawner (inactive by default)
    velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controller", "--inactive"],
        output="screen",
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # RViz2 for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription(
        declared_arguments + [
            ptz_state_publisher,
            control_node,
            joint_state_broadcaster_spawner,
            position_controller_spawner,
            velocity_controller_spawner,
            rviz_node,
        ]
    )