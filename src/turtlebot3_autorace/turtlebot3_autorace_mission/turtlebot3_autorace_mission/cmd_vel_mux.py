#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class CmdVelMux(Node):
    def __init__(self):
        super().__init__('cmd_vel_mux')
        
        # State
        self.navigation_active = False
        self.lane_cmd = Twist()
        self.nav_cmd = Twist()
        self.last_lane_time = self.get_clock().now()
        self.last_nav_time = self.get_clock().now()
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers
        self.lane_cmd_sub = self.create_subscription(
            Twist, '/lane_cmd_vel', self.lane_cmd_callback, 10)
        self.nav_cmd_sub = self.create_subscription(
            Twist, '/cmd_vel_nav', self.nav_cmd_callback, 10)
        self.nav_status_sub = self.create_subscription(
            Bool, '/navigation_active', self.nav_status_callback, 10)
            
        # Timer for command arbitration
        self.timer = self.create_timer(0.1, self.mux_commands)  # 10Hz
        
        self.get_logger().info('Command Velocity Multiplexer started')
        
    def lane_cmd_callback(self, msg):
        """Receive lane following commands"""
        self.lane_cmd = msg
        self.last_lane_time = self.get_clock().now()
        
    def nav_cmd_callback(self, msg):
        """Receive navigation commands"""
        self.nav_cmd = msg  
        self.last_nav_time = self.get_clock().now()
        
    def nav_status_callback(self, msg):
        """Navigation status callback"""
        prev_state = self.navigation_active
        self.navigation_active = msg.data
        
        if prev_state != self.navigation_active:
            status = "ACTIVE" if self.navigation_active else "INACTIVE"
            self.get_logger().info(f'Navigation status: {status}')
        
    def mux_commands(self):
        """Multiplex commands based on navigation state and freshness"""
        current_time = self.get_clock().now()
        
        # Check command freshness (1 second timeout)
        lane_fresh = (current_time - self.last_lane_time).nanoseconds / 1e9 < 1.0
        nav_fresh = (current_time - self.last_nav_time).nanoseconds / 1e9 < 1.0
        
        output_cmd = Twist()  # Default: stop
        
        if self.navigation_active and nav_fresh:
            # Navigation mode with fresh commands
            output_cmd = self.nav_cmd
            source = "NAV"
        elif not self.navigation_active and lane_fresh:
            # Lane following mode with fresh commands  
            output_cmd = self.lane_cmd
            source = "LANE"
        else:
            # No fresh commands - send stop
            source = "STOP"
            
        # Publish the selected command
        self.cmd_vel_pub.publish(output_cmd)
        
        # Debug output every 2 seconds
        if hasattr(self, '_last_debug_time'):
            if (current_time - self._last_debug_time).nanoseconds / 1e9 > 2.0:
                self._debug_output(source)
        else:
            self._last_debug_time = current_time
            
    def _debug_output(self, source):
        """Debug information output"""  
        self._last_debug_time = self.get_clock().now()
        
        self.get_logger().info(
            f'Mux: {source} | Nav: {self.navigation_active} | '
            f'Lane: ({self.lane_cmd.linear.x:.2f}, {self.lane_cmd.angular.z:.2f}) | '
            f'Nav: ({self.nav_cmd.linear.x:.2f}, {self.nav_cmd.angular.z:.2f})'
        )

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMux()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()