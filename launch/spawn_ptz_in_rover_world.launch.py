from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, PathJoinSubstitution, PythonExpression
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
import os
import os


def generate_launch_description():
    # Get package path
    package_path = get_package_share_directory('ptz_gz_sim')
    
    # Plugin path for CameraZoomPlugin 
    plugin_path = os.path.join(package_path, 'plugins')

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
    declared_arguments.append(
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run Gazebo in headless mode (server only) - NOT USED, world should already exist'
        )
    )

    # Configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    ptz_x = LaunchConfiguration('ptz_x')
    ptz_y = LaunchConfiguration('ptz_y') 
    ptz_z = LaunchConfiguration('ptz_z')
    headless = LaunchConfiguration('headless')

    # Path to Xacro file
    xacro_path = os.path.join(package_path, 'urdf', 'axis_camera_gazebo.xacro')

    # PTZ description
    ptz_description = {"robot_description": 
                      ParameterValue(Command(['xacro ', xacro_path]), value_type=str)}
   
    # NOTE: Gazebo world leonardo_race should already be running from rover motion stack
    # This launch only spawns PTZ in existing world - NO ROS2 Control needed, direct Gazebo plugins

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
        additional_env={'GZ_SIM_PLUGIN_PATH': plugin_path + ':/opt/ros/humble/lib:' + os.environ.get('GZ_SIM_PLUGIN_PATH', '')},
        output='screen'
    )

    # Robot State Publisher for PTZ (for RViz visualization)
    ptz_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher', 
        name='ptz_state_publisher',
        output='screen',
        parameters=[ptz_description, {'use_sim_time': use_sim_time}],
    )

    # Fake axis camera node (handles both Gazebo direct control + RViz joint_states)
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
            # Zoom topics re-enabled for full functionality
            "/ptz_zoom_cmd@std_msgs/msg/Float64@gz.msgs.Double",
            "/ptz_zoom_fb@std_msgs/msg/Float64@gz.msgs.Double",
            # Working joint control topics (same as minimal_sim)
            "/pan_cmd@std_msgs/msg/Float64@gz.msgs.Double",
            "/tilt_cmd@std_msgs/msg/Float64@gz.msgs.Double",
            # Frame bridges (MISSING BEFORE)
            # "/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V", 
            # "/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock",
        ],
        additional_env={'GZ_SIM_PLUGIN_PATH': plugin_path + ':/opt/ros/humble/lib:' + os.environ.get('GZ_SIM_PLUGIN_PATH', '')},
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription(
        declared_arguments + [
            # NOTE: NO gz_sim launch - world should already exist
            # NO ROS2 Control - direct Gazebo plugin approach like minimal_sim
            ptz_state_publisher,
            gz_spawn_ptz,
            fake_axis_camera,
            gz_ros2_bridge_ptz,
        ]
    )