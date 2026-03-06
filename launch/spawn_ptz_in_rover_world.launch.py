from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
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
    declared_arguments.append(
        DeclareLaunchArgument(
            'ptz_x',
            default_value='2.0',
            description='X position of PTZ camera in world'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'ptz_y', 
            default_value='0.0',
            description='Y position of PTZ camera in world'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'ptz_z',
            default_value='2.0',
            description='Z position of PTZ camera in world'
        )
    )

    # Configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    ptz_x = LaunchConfiguration('ptz_x')
    ptz_y = LaunchConfiguration('ptz_y') 
    ptz_z = LaunchConfiguration('ptz_z')

    # Path to Xacro file
    xacro_path = os.path.join(package_path, 'urdf', 'axis_camera_gazebo.xacro')

    # PTZ description
    ptz_description = {"robot_description": 
                      ParameterValue(Command(['xacro ', xacro_path]), value_type=str)}
   
    # Controller config
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("ptz_gz_sim"), "config", "ptz_controllers.yaml"]
    )

    # Spawn PTZ Camera in existing leonardo_race world
    gz_spawn_ptz = Node(
        package="ros_gz_sim",
        executable="create",
        name='ptz_spawner',
        arguments=[
            "-world", "leonardo_race",
            "-topic", "/robot_description",
            "-name", "ptz_camera",
            "-x", ptz_x,
            "-y", ptz_y, 
            "-z", ptz_z,
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
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

    # Fake axis camera node
    fake_axis_camera = Node(
        package='ptz_gz_sim',
        executable='fake_axis_camera_node',
        name='fake_axis_camera_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Bridge for PTZ camera topics  
    gz_ros2_bridge_ptz = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/image_raw@sensor_msgs/msg/Image@gz.msgs.Image",
            "/image_raw/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            # Zoom topics commented out since zoom plugin is disabled
            # "/ptz_zoom_cmd@std_msgs/msg/Float64@gz.msgs.Double",
            # "/ptz_zoom_fb@std_msgs/msg/Float64@gz.msgs.Double",
        ],
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
            gz_spawn_ptz,
            fake_axis_camera,
            gz_ros2_bridge_ptz,
        ]
    )