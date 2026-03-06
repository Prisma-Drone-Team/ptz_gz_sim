#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Float64MultiArray
from ptz_action_server_msgs.msg import Ptz
from ptz_action_server_msgs.action import PtzMove
from ptz_action_server_msgs.msg import PtzState
import time


class PTZControllerTester(Node):
    def __init__(self):
        super().__init__('ptz_controller_tester')
        
        # Publishers for direct controller commands (bypass fake_axis_camera)
        self.pos_cmd_pub = self.create_publisher(
            Float64MultiArray, 
            '/position_controller/commands', 
            10
        )
        self.vel_cmd_pub = self.create_publisher(
            Float64MultiArray, 
            '/velocity_controller/commands', 
            10
        )
        
        # Publishers for PTZ messages (via fake_axis_camera)
        self.velocity_pub = self.create_publisher(
            Ptz,
            '/cmd/velocity',
            10
        )
        
        # Action client for position commands
        self.position_action_client = ActionClient(
            self, 
            PtzMove, 
            'move_ptz/position_abs'
        )
        
        # Subscriber to PTZ state
        self.ptz_state_sub = self.create_subscription(
            PtzState,
            '/ptz_state',
            self.ptz_state_callback,
            10
        )
        
        self.get_logger().info('PTZ Controller Tester Node Started')
        self.get_logger().info('Available commands:')
        self.get_logger().info('- test_velocity(): Test velocity control via /cmd/velocity')
        self.get_logger().info('- test_position(): Test position control via action')
        self.get_logger().info('- test_direct_pos(): Test direct position controller')
        self.get_logger().info('- test_direct_vel(): Test direct velocity controller')
        
        self.current_state = None
    
    def ptz_state_callback(self, msg):
        self.current_state = msg
        mode_str = "IDLE" if msg.mode == PtzState.MODE_IDLE else \
                  "VELOCITY" if msg.mode == PtzState.MODE_VELOCITY else \
                  "POSITION" if msg.mode == PtzState.MODE_POSITION else "UNKNOWN"
        
        self.get_logger().info(
            f'PTZ State - Mode: {mode_str}, Pan: {msg.pan:.2f}, Tilt: {msg.tilt:.2f}, Zoom: {msg.zoom:.2f}'
        )
    
    def test_velocity(self):
        """Test velocity control via fake_axis_camera"""
        self.get_logger().info('=== TESTING VELOCITY CONTROL ===')
        
        # Send velocity commands
        vel_msg = Ptz()
        vel_msg.pan = 0.5    # rad/s
        vel_msg.tilt = -0.2  # rad/s  
        vel_msg.zoom = 0.0
        
        self.get_logger().info(f'Sending velocity command: pan={vel_msg.pan}, tilt={vel_msg.tilt}')
        self.velocity_pub.publish(vel_msg)
        
        time.sleep(2.0)
        
        # Stop movement
        vel_msg.pan = 0.0
        vel_msg.tilt = 0.0
        self.velocity_pub.publish(vel_msg)
        self.get_logger().info('Stopped velocity command')
    
    def test_position(self):
        """Test position control via action server"""
        self.get_logger().info('=== TESTING POSITION CONTROL ===')
        
        if not self.position_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available!')
            return
        
        # Create position goal
        goal_msg = PtzMove.Goal()
        goal_msg.ptz.pan = 1.0   # rad
        goal_msg.ptz.tilt = -0.5 # rad
        goal_msg.ptz.zoom = 1.0
        
        self.get_logger().info(f'Sending position command: pan={goal_msg.ptz.pan}, tilt={goal_msg.ptz.tilt}')
        
        # Send goal
        send_goal_future = self.position_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return
        
        self.get_logger().info('Goal accepted, waiting for result...')
        
        # Wait for result
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        
        result = get_result_future.result().result
        self.get_logger().info(f'Position control completed: {result.message}')
    
    def test_direct_pos(self):
        """Test direct position controller (bypass fake_axis_camera)"""
        self.get_logger().info('=== TESTING DIRECT POSITION CONTROLLER ===')
        
        pos_msg = Float64MultiArray()
        pos_msg.data = [0.8, 0.3]  # pan, tilt in radians
        
        self.get_logger().info(f'Sending direct position command: {pos_msg.data}')
        self.pos_cmd_pub.publish(pos_msg)
    
    def test_direct_vel(self):
        """Test direct velocity controller (bypass fake_axis_camera)"""
        self.get_logger().info('=== TESTING DIRECT VELOCITY CONTROLLER ===')
        
        vel_msg = Float64MultiArray()
        vel_msg.data = [0.3, -0.2]  # pan_vel, tilt_vel in rad/s
        
        self.get_logger().info(f'Sending direct velocity command: {vel_msg.data}')
        self.vel_cmd_pub.publish(vel_msg)
        
        time.sleep(2.0)
        
        # Stop
        vel_msg.data = [0.0, 0.0]
        self.vel_cmd_pub.publish(vel_msg)
        self.get_logger().info('Stopped direct velocity command')


def main(args=None):
    rclpy.init(args=args)
    
    tester = PTZControllerTester()
    
    try:
        # Keep node alive for subscribers
        rclpy.spin(tester)
    except KeyboardInterrupt:
        tester.get_logger().info('Shutting down...')
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()