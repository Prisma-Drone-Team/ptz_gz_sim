#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch file for PTZ Navigation Stack (hardware motion stack adapted for simulation)
    
    This launch file integrates the hardware ptz_manager with the simulation
    by remapping topics and launching supporting nodes.
    
    Run this AFTER launching the PTZ simulation with spawn_ptz_in_rover_world.launch.py
    """
    
    # Package directories
    ptz_manager_dir = get_package_share_directory('ptz_manager')
    
    # Launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )
    
    use_yolo_arg = DeclareLaunchArgument(
        'use_yolo',
        default_value='false',  # Default false since it requires GPU
        description='Launch YOLO ROI calculation module'
    )
    
    use_zbar_arg = DeclareLaunchArgument(
        'use_zbar',
        default_value='true',
        description='Launch ZBar QR code scanner'
    )

    # PTZ Manager node with topic remapping for simulation compatibility
    ptz_manager_node = Node(
        package='ptz_manager',
        executable='ptz_manager',
        name='ptz_manager',
        parameters=[
            os.path.join(ptz_manager_dir, 'param', 'param.yaml')
        ],
        remappings=[
            # Map hardware topics to simulation topics
            ('/axis/state', '/ptz_state'),           # Direct mapping to simulation state
            # Keep these unchanged - already compatible
            ('/cmd/velocity', '/cmd/velocity'),      # Direct compatibility
            ('cmd/ptz', 'cmd/ptz'),                  # Manual commands
            ('/seed_pdt_camera/command', '/seed_pdt_camera/command'),  # SEED commands
            ('/yolo/detections', '/yolo/detections'), # YOLO ROI
            ('/barcode', '/barcode'),                # QR codes
        ],
        output='screen'
    )

    # Image republisher (compressed -> raw) for YOLO
    image_republisher = Node(
        package='image_transport',
        executable='republish',
        name='image_republisher',
        arguments=['compressed', 'raw'],
        remappings=[
            ('in/compressed', '/camera/image_raw/compressed'),
            ('out', '/image_raw')
        ],
        condition=IfCondition(LaunchConfiguration('use_yolo')),
        output='screen'
    )

    # ZBar QR code scanner
    zbar_scanner = Node(
        package='zbar_ros',
        executable='barcode_reader',
        name='zbar_scanner',
        remappings=[
            ('/image', '/image_raw')
        ],
        condition=IfCondition(LaunchConfiguration('use_zbar')),
        output='screen'
    )

    # YOLO ROI calculation
    yolo_node = Node(
        package='roi_calculation_yolo',
        executable='roi_calculation_yolo',
        name='roi_calculation_yolo',
        remappings=[
            ('/image_raw', '/image_raw'),
            ('/yolo/detections', '/yolo/detections')
        ],
        condition=IfCondition(LaunchConfiguration('use_yolo')),
        output='screen'
    )

    # RViz for visualization (optional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_ptz_navigation',
        arguments=['-d', os.path.join(ptz_manager_dir, 'rviz', 'rviz.rviz')],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen'
    )

    return LaunchDescription([
        # Launch arguments
        use_rviz_arg,
        use_yolo_arg, 
        use_zbar_arg,
        
        # Core navigation stack
        ptz_manager_node,
        
        # Supporting nodes
        image_republisher,
        zbar_scanner,
        yolo_node,
        rviz_node,
    ])