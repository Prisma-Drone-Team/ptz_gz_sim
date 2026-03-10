import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import xacro


def generate_launch_description():

    pkg_share = get_package_share_directory("ptz_gz_sim")

    xacro_file = os.path.join(pkg_share, "urdf", "axis_camera_gazebo.xacro")

    doc = xacro.process_file(xacro_file)
    robot_desc = doc.toxml()

    # Launch arguments
    declare_headless_cmd = DeclareLaunchArgument(
        'headless',
        default_value='true',
        description='Launch Gazebo without GUI (headless mode)'
    )
    
    # Configuration variables
    headless = LaunchConfiguration('headless')

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_desc}],
        output="screen",
    )

    # Gazebo sim with headless support
    from launch.substitutions import PythonExpression
    
    gz_args = PythonExpression([
        "'-r empty.sdf -s' if '", headless, "' == 'true' else '-r empty.sdf'"
    ])
    
    gz_sim = ExecuteProcess(
        cmd=["gz", "sim", gz_args],
        output="screen",
        shell=True
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-string", robot_desc,
            "-name", "axis_camera",
            "-z", "0.1"
        ],
        output="screen",
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/pan_cmd@std_msgs/msg/Float64@gz.msgs.Double",
            "/tilt_cmd@std_msgs/msg/Float64@gz.msgs.Double",
        ],
        output="screen",
    )

    return LaunchDescription([
        declare_headless_cmd,
        gz_sim,
        robot_state_publisher,
        spawn,
        bridge,
    ])