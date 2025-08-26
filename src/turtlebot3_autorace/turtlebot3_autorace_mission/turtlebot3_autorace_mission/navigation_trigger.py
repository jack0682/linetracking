#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, UInt8
from nav2_simple_commander.robot_navigator import BasicNavigator
import math

class NavigationTrigger(Node):
    def __init__(self):
        super().__init__('navigation_trigger')
        
        # Trigger coordinates in odom frame (navigation 시작해야 하는 지점 - 진짜 시작 위치)
        self.trigger_x_odom = -1.7603088878461883  # 실제 진짜 시작 위치 (박스 내부)
        self.trigger_y_odom = -0.18501192976186576
        
        # Gazebo starting coordinates in odom frame (시작 지점에서도 트리거 가능)
        self.gazebo_start_x = 0.0  # Gazebo 시작 좌표
        self.gazebo_start_y = 0.0
        
        # Map coordinates (odom 좌표와 일치하게 설정)
        self.map_start_x = self.trigger_x_odom     # 맵 좌표 = odom 좌표 (맵 정렬을 위해)
        self.map_start_y = self.trigger_y_odom
        self.map_start_z = 0.008508788515588466
        self.map_start_yaw = -1.556020  # 시작 방향
        
        self.target_x_map = -0.045232   # 맵에서의 목표 지점 (문 위치) - 기존 맵 좌표 유지
        self.target_y_map = -1.744123
        self.target_z = 0.008545
        self.target_yaw = -0.012973     # 목표 방향
        
        # Navigation 완료 후 lane tracking 복귀 지점 (odom 좌표)
        self.lane_return_x_odom = -0.12053760049227365  # lane tracking 복귀 지점
        self.lane_return_y_odom = -1.760713242146403
        
        # Threshold for position matching (meters)
        self.position_threshold = 0.5  # 일반 트리거 지점 threshold
        self.start_position_threshold = 1.0  # 시작 지점 근처에서는 더 넓게
        
        # Navigation state
        self.navigation_triggered = False
        self.navigation_active = False
        self.initial_pose_set = False
        
        # Lane detection state
        self.current_lane_state = 0  # 현재 lane state (0=no lane detected)
        
        # Navigation control flags
        self.force_move_to_start = False  # 시작 위치로 강제 이동 플래그
        
        # Initialize navigator
        self.navigator = None
        
        # Subscribers
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        self.navigation_enable_sub = self.create_subscription(
            Bool,
            '/navigation_enable',
            self.navigation_enable_callback,
            10
        )
        
        # Lane state subscriber
        self.lane_state_sub = self.create_subscription(
            UInt8,
            '/detect/lane_state',
            self.lane_state_callback,
            10
        )
        
        # Publishers
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )
        
        self.nav_cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel_nav',  # Modified: 2025-08-26 12:00 - Route to mux
            10
        )
        
        # Subscribe to nav2 cmd_vel and relay it
        self.nav2_cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.nav2_cmd_callback,
            10
        )
        
        self.nav_status_pub = self.create_publisher(
            Bool,
            '/navigation_active',
            10
        )
        
        self.lane_control_disable_pub = self.create_publisher(
            Bool,
            '/lane_control_disable',
            10
        )
        
        # Lane following restart publisher
        self.lane_restart_pub = self.create_publisher(
            Bool,
            '/lane_following_restart',
            10
        )
        
        self.get_logger().info(f'Navigation trigger node started.')
        self.get_logger().info(f'Trigger conditions: 1) Main trigger at odom({self.trigger_x_odom:.2f}, {self.trigger_y_odom:.2f}) OR 2) Start position({self.gazebo_start_x}, {self.gazebo_start_y}) with lane_state=0')
        self.get_logger().info(f'Navigation path: odom({self.trigger_x_odom:.2f}, {self.trigger_y_odom:.2f}) → map({self.target_x_map}, {self.target_y_map}) → return to odom({self.lane_return_x_odom:.2f}, {self.lane_return_y_odom:.2f})')
        
    def lane_state_callback(self, msg):
        """Lane state callback to track current lane detection status"""
        self.current_lane_state = msg.data
        
    def odom_callback(self, msg):
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        
        if self.navigation_triggered:
            return  # 이미 트리거됨
            
        # Calculate distance to main trigger point in odom frame
        distance_to_trigger = math.sqrt((current_x - self.trigger_x_odom)**2 + (current_y - self.trigger_y_odom)**2)
        
        # Calculate distance to Gazebo starting position
        distance_to_start = math.sqrt((current_x - self.gazebo_start_x)**2 + (current_y - self.gazebo_start_y)**2)
        
        # 조건 1: 일반 트리거 지점에 도달 (기존 조건)
        if distance_to_trigger < self.position_threshold:
            self.get_logger().info(f'Robot reached main trigger position! Distance: {distance_to_trigger:.2f}m')
            self.trigger_navigation()
            return
            
        # 조건 2: 시작 지점 근처에서 lane_state가 0인 경우 (새로운 조건)
        if (distance_to_start < self.start_position_threshold and self.current_lane_state == 0):
            self.get_logger().info(f'Robot near start position with no lane detected! Distance: {distance_to_start:.2f}m, Lane state: {self.current_lane_state}')
            self.trigger_navigation()
            
    def navigation_enable_callback(self, msg):
        """External trigger for navigation enable"""
        if msg.data and not self.navigation_triggered:
            self.trigger_navigation()
            
    def nav2_cmd_callback(self, msg):
        """Relay nav2 commands to mux when navigation is active"""
        if self.navigation_active:
            self.nav_cmd_pub.publish(msg)
            
    def trigger_navigation(self):
        """Trigger navigation when robot reaches the specified position"""
        self.navigation_triggered = True
        
        # First, set initial pose for navigation
        self.set_initial_pose()
        
        # Check if robot needs to move to start position first
        self.force_move_to_start = True
        
        # Wait a moment for initial pose to be processed, then move to start
        self.create_timer(2.0, self.move_to_start_position)
        
    def set_initial_pose(self):
        """Set initial pose for navigation in map frame"""
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        
        # Set map coordinates as initial pose (맵에서의 시작점)
        initial_pose.pose.pose.position.x = self.map_start_x
        initial_pose.pose.pose.position.y = self.map_start_y
        initial_pose.pose.pose.position.z = self.map_start_z
        
        # Set orientation using start yaw
        yaw = self.map_start_yaw
        initial_pose.pose.pose.orientation.x = 0.0
        initial_pose.pose.pose.orientation.y = 0.0
        initial_pose.pose.pose.orientation.z = math.sin(yaw / 2.0)
        initial_pose.pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        # Set covariance
        initial_pose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                                       0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                       0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909]
        
        self.initial_pose_pub.publish(initial_pose)
        self.initial_pose_set = True
        self.get_logger().info(f'Initial pose set in map frame: ({self.map_start_x}, {self.map_start_y})')
        
    def move_to_start_position(self):
        """Move robot to the navigation start position first"""
        try:
            # Initialize navigator if not done
            if self.navigator is None:
                self.navigator = BasicNavigator()
                self.navigator.waitUntilNav2Active()
            
            # Create goal pose for start position (inside the box)
            start_pose = PoseStamped()
            start_pose.header.frame_id = 'map'
            start_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            
            # Set start position in map frame (where robot should be positioned)
            start_pose.pose.position.x = self.map_start_x
            start_pose.pose.position.y = self.map_start_y
            start_pose.pose.position.z = self.map_start_z
            
            # Set start orientation
            yaw = self.map_start_yaw
            start_pose.pose.orientation.x = 0.0
            start_pose.pose.orientation.y = 0.0
            start_pose.pose.orientation.z = math.sin(yaw / 2.0)
            start_pose.pose.orientation.w = math.cos(yaw / 2.0)
            
            # Navigate to start position first
            self.navigator.goToPose(start_pose)
            self.navigation_active = True
            
            # Publish navigation active status
            nav_status = Bool()
            nav_status.data = True
            self.nav_status_pub.publish(nav_status)
            
            self.get_logger().info(f'Moving to start position first: ({self.map_start_x}, {self.map_start_y})')
            
            # Monitor movement to start position
            self.create_timer(1.0, self.monitor_start_movement)
            
        except Exception as e:
            self.get_logger().error(f'Failed to move to start position: {str(e)}')
            
    def monitor_start_movement(self):
        """Monitor movement to start position"""
        if self.navigator and not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                self.get_logger().info(f'Distance to start position: {feedback.distance_remaining:.2f}m')
        elif self.navigator and self.navigator.isTaskComplete():
            result = self.navigator.getResult()
            if result == BasicNavigator.TaskResult.SUCCEEDED:
                self.get_logger().info('Successfully reached start position! Now proceeding to target.')
                # Now start the actual navigation to target
                self.create_timer(1.0, self.start_navigation)
            else:
                self.get_logger().error('Failed to reach start position')
            
            # Stop monitoring start movement
            return False
        return True
        
    def start_navigation(self):
        """Start navigation to target position"""
        try:
            # Initialize navigator if not done
            if self.navigator is None:
                self.navigator = BasicNavigator()
                self.navigator.waitUntilNav2Active()
            
            # Create goal pose
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            
            # Set target position in map frame
            goal_pose.pose.position.x = self.target_x_map
            goal_pose.pose.position.y = self.target_y_map
            goal_pose.pose.position.z = self.target_z
            
            # Set target orientation
            yaw = self.target_yaw
            goal_pose.pose.orientation.x = 0.0
            goal_pose.pose.orientation.y = 0.0
            goal_pose.pose.orientation.z = math.sin(yaw / 2.0)
            goal_pose.pose.orientation.w = math.cos(yaw / 2.0)
            
            # Navigate to goal
            self.navigator.goToPose(goal_pose)
            self.navigation_active = True
            
            # Publish navigation active status
            nav_status = Bool()
            nav_status.data = True
            self.nav_status_pub.publish(nav_status)
            
            self.get_logger().info(f'Navigation started to map target: ({self.target_x_map}, {self.target_y_map})')
            
            # Monitor navigation progress
            self.create_timer(1.0, self.monitor_navigation)
            
        except Exception as e:
            self.get_logger().error(f'Failed to start navigation: {str(e)}')
            
    def monitor_navigation(self):
        """Monitor navigation progress"""
        if self.navigator and not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                self.get_logger().info(f'Distance to goal: {feedback.distance_remaining:.2f}m')
        elif self.navigator and self.navigator.isTaskComplete():
            result = self.navigator.getResult()
            if result == BasicNavigator.TaskResult.SUCCEEDED:
                self.get_logger().info('Successfully reached the door!')
            else:
                self.get_logger().info('Failed to reach the door')
            
            # Navigation completed - disable navigation mode
            self.navigation_active = False
            nav_status = Bool()
            nav_status.data = False
            self.nav_status_pub.publish(nav_status)
            
            self.get_logger().info(f'Navigation completed - Lane control re-enabled at odom({self.lane_return_x_odom:.2f}, {self.lane_return_y_odom:.2f})')
            
            # Lane tracking 완전 복귀 로직 구현
            self.restart_lane_following()
            
            # Stop monitoring
            return False
        return True
    
    def restart_lane_following(self):
        """완전한 lane tracking 복귀 로직"""
        try:
            # 1. Lane following restart 신호 발행
            restart_msg = Bool()
            restart_msg.data = True
            self.lane_restart_pub.publish(restart_msg)
            
            # 2. Lane control disable 해제
            disable_msg = Bool()
            disable_msg.data = False
            self.lane_control_disable_pub.publish(disable_msg)
            
            # 3. Navigation 상태 완전 리셋
            self.navigation_triggered = False
            self.navigation_active = False
            self.initial_pose_set = False
            self.force_move_to_start = False
            
            self.get_logger().info('Lane following restart signals published:')
            self.get_logger().info(f'  - /lane_following_restart: True')
            self.get_logger().info(f'  - /lane_control_disable: False')
            self.get_logger().info(f'  - /navigation_active: False')
            self.get_logger().info('System ready for next navigation cycle')
            
        except Exception as e:
            self.get_logger().error(f'Failed to restart lane following: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = NavigationTrigger()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()