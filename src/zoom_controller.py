#!/usr/bin/env python3

"""
Simple zoom controller that can change camera FOV via Gazebo services.
This provides an alternative to the CameraZoomPlugin for controlling camera zoom.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float64
from gz.transport import Node as GzNode
from gz.msgs import Double
import math

class ZoomController(Node):
    def __init__(self):
        super().__init__('zoom_controller')
        
        # Parameters
        self.declare_parameter('camera_name', 'ptz_camera')
        self.declare_parameter('min_fov', 0.1)  # radians
        self.declare_parameter('max_fov', 1.5)  # radians  
        self.declare_parameter('default_fov', 1.0786)  # radians (about 62 degrees)
        
        self.camera_name = self.get_parameter('camera_name').value
        self.min_fov = self.get_parameter('min_fov').value
        self.max_fov = self.get_parameter('max_fov').value
        self.default_fov = self.get_parameter('default_fov').value
        self.current_fov = self.default_fov
        
        # ROS subscribers
        self.zoom_sub = self.create_subscription(
            Float64,
            '/ptz_zoom_cmd',
            self.zoom_callback,
            10
        )
        
        # ROS publishers
        self.zoom_fb_pub = self.create_publisher(Float64, '/ptz_zoom_fb', 10)
        
        # Gazebo transport node
        self.gz_node = GzNode()
        
        # Publisher to set camera FOV - we'll try different topic patterns
        self.fov_pub = None
        self.setup_gazebo_publishers()
        
        # Timer for feedback
        self.timer = self.create_timer(0.1, self.publish_feedback)
        
        self.get_logger().info(f"Zoom controller initialized for camera: {self.camera_name}")
        
    def setup_gazebo_publishers(self):
        """Setup Gazebo publishers for camera FOV control"""
        # Try different topic patterns that Gazebo might use
        possible_topics = [
            f"/model/axis_camera/sensor/{self.camera_name}/cmd_fov",
            f"/sensor/{self.camera_name}/cmd_fov", 
            f"/{self.camera_name}/cmd_fov",
            "/axis_camera/ptz_camera/cmd_fov",
            "/cmd_fov"
        ]
        
        for topic in possible_topics:
            try:
                self.fov_pub = self.gz_node.advertise(topic, Double)
                self.get_logger().info(f"Successfully created FOV publisher on topic: {topic}")
                break
            except Exception as e:
                self.get_logger().debug(f"Failed to create publisher on {topic}: {e}")
                continue
                
        if not self.fov_pub:
            self.get_logger().warn("Could not create any FOV publisher. Will try service calls.")
    
    def zoom_callback(self, msg):
        """Handle zoom command (zoom factor: 1.0 = no zoom, 2.0 = 2x zoom, etc.)"""
        zoom_factor = msg.data
        
        # Clamp zoom factor to reasonable limits
        zoom_factor = max(1.0, min(10.0, zoom_factor))
        
        # Convert zoom factor to FOV (inverse relationship)
        new_fov = self.default_fov / zoom_factor
        new_fov = max(self.min_fov, min(self.max_fov, new_fov))
        
        self.set_camera_fov(new_fov)
        self.current_fov = new_fov
        
        self.get_logger().info(f"Zoom command: {zoom_factor}x, FOV: {math.degrees(new_fov):.1f}°")
    
    def set_camera_fov(self, fov_radians):
        """Set camera FOV via Gazebo transport"""
        if self.fov_pub:
            msg = Double()
            msg.data = fov_radians
            self.fov_pub.publish(msg)
        else:
            self.get_logger().warn("No FOV publisher available")
    
    def publish_feedback(self):
        """Publish current zoom feedback"""
        zoom_factor = self.default_fov / self.current_fov
        msg = Float64()
        msg.data = zoom_factor
        self.zoom_fb_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ZoomController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()