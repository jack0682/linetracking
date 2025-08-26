#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry
import math

class ControlArbiter(Node):
    def __init__(self):
        super().__init__('control_arbiter')
        
        # Control states
        self.current_controller = 'LANE_FOLLOWING'  # LANE_FOLLOWING, NAVIGATION
        self.navigation_active = False
        self.navigation_triggered = False
        
        # Position tracking
        self.current_x = 0.0
        self.current_y = 0.0
        self.trigger_x_odom = -2.47
        self.trigger_y_odom = 1.67
        self.position_threshold = 0.5
        
        # Command storage
        self.lane_cmd = Twist()
        self.nav_cmd = Twist()
        self.last_cmd_time = {}
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.control_status_pub = self.create_publisher(String, '/control_status', 10)
        self.navigation_enable_pub = self.create_publisher(Bool, '/navigation_enable', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.lane_cmd_sub = self.create_subscription(Twist, '/lane_cmd_vel', self.lane_cmd_callback, 10)
        self.nav_cmd_sub = self.create_subscription(Twist, '/nav_cmd_vel', self.nav_cmd_callback, 10)
        self.nav_status_sub = self.create_subscription(Bool, '/navigation_active', self.nav_status_callback, 10)
        
        # Timer for control arbitration
        self.control_timer = self.create_timer(0.1, self.arbitrate_control)  # 10Hz
        
        self.get_logger().info('Control Arbiter started - Managing LANE_FOLLOWING vs NAVIGATION')
        
    def odom_callback(self, msg):
        """Track robot position for navigation trigger"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Check if robot reached trigger point
        distance = math.sqrt((self.current_x - self.trigger_x_odom)**2 + 
                           (self.current_y - self.trigger_y_odom)**2)
        
        if distance < self.position_threshold and not self.navigation_triggered:
            self.navigation_triggered = True
            self.get_logger().info(f'Navigation trigger activated! Distance: {distance:.2f}m')
            
            # Enable navigation
            enable_msg = Bool()
            enable_msg.data = True
            self.navigation_enable_pub.publish(enable_msg)
            
    def lane_cmd_callback(self, msg):
        """Receive commands from lane following controller"""
        self.lane_cmd = msg
        self.last_cmd_time['lane'] = self.get_clock().now()
        
    def nav_cmd_callback(self, msg):
        """Receive commands from navigation controller"""
        self.nav_cmd = msg
        self.last_cmd_time['nav'] = self.get_clock().now()
        
    def nav_status_callback(self, msg):
        """Receive navigation active status"""
        self.navigation_active = msg.data
        if msg.data:
            self.current_controller = 'NAVIGATION'
            self.get_logger().info('Switched to NAVIGATION control')
        elif self.navigation_triggered:
            # Navigation completed, stay in navigation mode or return to lane following
            self.current_controller = 'LANE_FOLLOWING'
            self.get_logger().info('Navigation completed - Returned to LANE_FOLLOWING')
            
    def arbitrate_control(self):
        """Main control arbitration logic"""
        current_time = self.get_clock().now()
        
        # Determine active controller
        if self.navigation_triggered and self.navigation_active:
            active_controller = 'NAVIGATION'
            active_cmd = self.nav_cmd
            cmd_topic = 'nav'
        else:
            active_controller = 'LANE_FOLLOWING'  
            active_cmd = self.lane_cmd
            cmd_topic = 'lane'
        
        # Check command freshness (timeout after 0.5 seconds)
        if cmd_topic in self.last_cmd_time:
            time_diff = (current_time - self.last_cmd_time[cmd_topic]).nanoseconds / 1e9
            if time_diff > 0.5:
                # Command too old, send stop command
                active_cmd = Twist()
                self.get_logger().warn(f'{active_controller} command timeout - Stopping robot')
        
        # Update controller if changed
        if self.current_controller != active_controller:
            self.current_controller = active_controller
            self.get_logger().info(f'Control switched to: {active_controller}')
            
        # Publish the active command
        self.cmd_vel_pub.publish(active_cmd)
        
        # Publish status
        status_msg = String()
        status_msg.data = f'{active_controller}|triggered:{self.navigation_triggered}|active:{self.navigation_active}'
        self.control_status_pub.publish(status_msg)
        
        # Debug output every 2 seconds
        if hasattr(self, '_last_debug_time'):
            if (current_time - self._last_debug_time).nanoseconds / 1e9 > 2.0:
                self._debug_output()
        else:
            self._last_debug_time = current_time
            
    def _debug_output(self):
        """Debug information output"""
        self._last_debug_time = self.get_clock().now()
        
        distance_to_trigger = math.sqrt((self.current_x - self.trigger_x_odom)**2 + 
                                      (self.current_y - self.trigger_y_odom)**2)
        
        self.get_logger().info(
            f'Controller: {self.current_controller} | '
            f'Pos: ({self.current_x:.2f}, {self.current_y:.2f}) | '
            f'Dist to trigger: {distance_to_trigger:.2f}m | '
            f'Nav triggered: {self.navigation_triggered} | '
            f'Nav active: {self.navigation_active}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = ControlArbiter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()