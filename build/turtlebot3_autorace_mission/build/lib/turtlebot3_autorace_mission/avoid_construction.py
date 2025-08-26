#!/usr/bin/env python3
#
# Copyright 2025 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: ChanHyeong Lee

import math
import random
from collections import deque
from scipy import signal

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_msgs.msg import UInt8
from std_msgs.msg import String
from geometry_msgs.msg import Point


def euler_from_quaternion(msg):
    """
    Convert a quaternion into Euler angles (roll, pitch, yaw).

    msg: geometry_msgs.msg.Quaternion.
    return: (roll, pitch, yaw) tuple.
    """
    x = msg.x
    y = msg.y
    z = msg.z
    w = msg.w
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = max(min(t2, 1.0), -1.0)
    pitch = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return roll, pitch, yaw


class AvoidConstruction(Node):

    def __init__(self):
        super().__init__('avoid_construction')

        # Subscribe
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )
        self.lane_state_sub = self.create_subscription(
            UInt8,
            '/detect/lane_state',
            self.lane_state_callback,
            10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_projected',
            self.image_callback,
            10
        )

        self.detect_traffic_type_sub = self.create_subscription(
            String,
            '/detect/traffic/type',
            self.traffic_type_callback,
            10
        )
        
        # [START] Code added for parking sign detection (2025-08-25)
        self.detect_parking_sign_sub = self.create_subscription(
            UInt8,
            '/detect/traffic_sign',
            self.parking_sign_callback,
            10
        )
        
        self.detect_parking_center_sub = self.create_subscription(
            Point,
            '/detect/parking_center',
            self.parking_center_callback,
            10
        )
        # [END] Code added for parking sign detection (2025-08-25)

        # Publish
        self.image_pub = self.create_publisher(
            Image,
            '/lane_detection/image_result',
            10
        )
        self.avoid_cmd_pub = self.create_publisher(
            Twist,
            '/lane_cmd_vel',  # Changed to arbiter topic
            10
        )
        self.avoid_active_pub = self.create_publisher(
            Bool,
            '/avoid_active',
            10
        )

        self.bridge = CvBridge()
        self.lane_detected = False
        

        # Parameter settings
        self.danger_distance = 0.24    # Danger zone y threshold (meters)
        self.danger_width = 0.12       # Danger zone x width (meters)
        self.speed = 0.03              # Forward speed during avoidance

        # Traffic light parameters
        self.traffic_light_distance = 0.5  # Distance to stop at traffic light (meters)
        self.traffic_light_detected_distance = 1.0  # Distance to start monitoring traffic light
        self.traffic_type = None  # Current traffic light state
        self.traffic_last_update_time = None  # Last time traffic signal was received
        self.traffic_signal_timeout = 2.0  # Timeout for traffic signal (seconds)
        self.yellow_speed_multiplier = 1.5  # Speed increase for yellow light
        self.traffic_light_active = False  # Whether traffic light control is active

        # Enhanced PD control parameters (for turning)
        self.turn_Kp = 0.45
        self.turn_Kd = 0.03
        self.turn_threshold_enter = 0.05   # Threshold to enter next state
        self.turn_threshold_exit = 0.08    # Threshold to exit current state (hysteresis)
        
        # Savitzky-Golay filter for derivative calculation
        self.sg_window_size = 9    # Must be odd
        self.sg_poly_order = 2
        self.error_history = deque(maxlen=self.sg_window_size)
        self.time_history = deque(maxlen=self.sg_window_size)
        
        # Filtered derivative and EWMA for state transitions
        self.filtered_derivative = 0.0
        self.lpf_alpha = 0.7  # Low-pass filter coefficient (0 < alpha < 1)
        self.ewma_error = 0.0
        self.ewma_alpha = 0.3  # EWMA coefficient
        
        # Angular velocity limits
        self.max_angular_velocity = 1.0  # rad/s
        self.max_angular_acceleration = 2.0  # rad/s²
        self.last_angular_velocity = 0.0
        self.last_time = None

        # State machine variables
        # States: 'NORMAL', 'AVOID_TURN', 'AVOID_STRAIGHT', 'RETURN_TURN', 'TRAFFIC_STOP', 'PARKING_MANEUVER'
        self.state = 'NORMAL'
        self.turn_direction = None     # 'left' or 'right'
        self.desired_theta = None      # Target angle (radians)
        self.original_theta = None     # Original heading at the start of avoidance mode

        # Current odom information
        self.current_theta = 0.0
        self.current_pos_x = 0.0
        self.current_pos_y = 0.0
        
        # [START] Waypoint-based parking system (2025-08-26)
        # Parking state management
        self.parking_sign_detected = False
        self.parking_last_update_time = None
        self.parking_signal_timeout = 3.0  # 신호 감지 timeout (사용 안 함)
        self.parking_maneuver_active = False
        self.parking_start_time = None  # 주차 시작 시간 기록
        self.parking_minimum_duration = 20.0  # 최소 주차 지속 시간 (20초)
        
        # Waypoint coordinates (odom frame) - (x, y, target_yaw)
        self.start_waypoint = (0.4936114648095425, 1.706690604705027, -1.556)     # 시작 지점
        self.middle_waypoint = (0.49450489458485963, 0.7432805201960125, -1.556)  # 중간 경유지  
        self.parking_space_1 = (0.7438981271471021, 0.7377510257783652, -1.556)   # 주차공간1
        self.parking_space_2 = (0.2648888284365445, 0.7373139655277864, -1.556)   # 주차공간2
        self.return_to_lane_waypoint = (0.1093812414389374, 1.7656589346728635, -1.564)  # 출차 후 lane 복귀 지점 (약 -π/2)
        
        # Parking navigation parameters
        self.waypoint_tolerance = 0.15  # Distance tolerance to consider waypoint reached
        self.parking_speed = 0.03       # Speed during parking navigation
        self.current_waypoint = None    # Current target waypoint
        self.selected_parking_space = None  # Randomly selected parking space
        self.parking_phase = 'IDLE'     # 'IDLE', 'TO_START', 'TO_MIDDLE', 'TO_PARKING', 'PARKED', 'DEPARKED'
        self.parking_stop_duration = 5.0  # Duration to stop at parking spot (seconds)
        self.parking_stop_start_time = None
        
        # Random parking space selection (done once when parking starts)
        self.parking_spaces = [self.parking_space_1, self.parking_space_2]
        # [END] Waypoint-based parking system (2025-08-26)

        # lane_state value: 1 (left lane only), 2 (both), 3 (right lane only), 4 (not detected)
        self.lane_state = None

        self.lidar_points = None
        self.front_distance = float('inf')  # Distance to closest front obstacle

        self.timer = self.create_timer(0.1, self.process_loop)
        
        # Initialize time tracking
        self.last_time = self.get_clock().now()

    def lidar_callback(self, msg):
        self.lidar_points = self.convert_laserscan_to_points(msg)
        self.front_distance = self.get_front_distance(msg)
        self.visualization(self.lidar_points)

    def convert_laserscan_to_points(self, msg):
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        ranges = np.array(msg.ranges)
        valid = (ranges >= msg.range_min) & (ranges <= msg.range_max)
        ranges = ranges[valid]
        angles = angles[valid]
        if len(ranges) == 0:
            return np.array([])
        x = ranges * -np.sin(angles)
        y = ranges * np.cos(angles)
        return np.vstack((x, y)).T

    def get_front_distance(self, msg):
        """Get the closest distance in front of the robot (±30 degrees)"""
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        ranges = np.array(msg.ranges)
        
        # Front sector: ±30 degrees (±π/6 radians)
        front_mask = (angles >= -math.pi/6) & (angles <= math.pi/6)
        front_ranges = ranges[front_mask]
        
        # Filter valid ranges
        valid_ranges = front_ranges[(front_ranges >= msg.range_min) & 
                                  (front_ranges <= msg.range_max)]
        
        if len(valid_ranges) > 0:
            return np.min(valid_ranges)
        else:
            return float('inf')

    def lane_state_callback(self, msg):
        self.lane_state = msg.data

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        euler = euler_from_quaternion(q)
        self.current_theta = euler[2]
        self.current_pos_x = msg.pose.pose.position.x
        self.current_pos_y = msg.pose.pose.position.y
    
    def traffic_type_callback(self, msg):
        self.traffic_type = msg.data
        self.traffic_last_update_time = self.get_clock().now()
        self.traffic_light_active = True
        self.get_logger().info(f'Traffic light detected: {self.traffic_type}')
    
    def publish_control_command(self, twist):
        """Publish control command to avoid topic"""
        self.avoid_cmd_pub.publish(twist)
    
    # [START] Waypoint-based parking callback (2025-08-26)
    def parking_sign_callback(self, msg):
        """Handle parking sign detection - start waypoint navigation"""
        if msg.data == 1 and not self.parking_maneuver_active:  # Parking sign detected
            self.parking_sign_detected = True
            self.parking_last_update_time = self.get_clock().now()
            self.parking_start_time = self.get_clock().now()  # 주차 시작 시간 기록
            self.parking_maneuver_active = True
            
            # Randomly select parking space
            self.selected_parking_space = random.choice(self.parking_spaces)
            space_num = 1 if self.selected_parking_space == self.parking_space_1 else 2
            
            # Start waypoint navigation
            self.parking_phase = 'TO_START'
            self.current_waypoint = self.start_waypoint
            
            self.get_logger().info(f'Parking sign detected! Selected parking space {space_num}')
            self.get_logger().info(f'Starting waypoint navigation: Phase {self.parking_phase}')
            self.get_logger().info(f'Parking will continue for minimum {self.parking_minimum_duration} seconds')
    
    def parking_center_callback(self, msg):
        """Handle parking sign center position (not used in waypoint system)"""
        pass  # Keep for compatibility but not used in waypoint navigation
    # [END] Waypoint-based parking callback (2025-08-26)

    def image_callback(self, msg):
        if self.state == 'NORMAL':
            return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            self.get_logger().error('Image conversion failed')
            return

        img_height, img_width = cv_image.shape[:2]

        # Define ROI
        roi_size = img_height
        roi_x_start = (img_width - roi_size) // 2
        roi_x_end = roi_x_start + roi_size
        roi = cv_image[0:roi_size, roi_x_start:roi_x_end]

        margin = 20

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, 50, 150)
        lines = cv2.HoughLinesP(
            edges, 1, np.pi/180, threshold=50, minLineLength=100, maxLineGap=10
        )

        positive_found = False
        negative_found = False

        if lines is not None:
            for line in lines:
                for x1, y1, x2, y2 in line:
                    # Calculate line length
                    line_length = np.hypot(x2 - x1, y2 - y1)
                    if line_length < 100:
                        continue
                    # Check if the line covers near the top and bottom of the ROI
                    if max(y1, y2) < margin or min(y1, y2) > (roi_size - margin):
                        continue

                    if (x2 - x1) == 0:
                        continue
                    slope = (y2 - y1) / (x2 - x1)
                    if slope > 0:
                        positive_found = True
                    elif slope < 0:
                        negative_found = True
            if positive_found and negative_found:
                self.lane_detected = True
            else:
                self.lane_detected = False
        else:
            self.lane_detected = False

        overlay_text = 'Lane Detected' if self.lane_detected else 'Lane Not Detected'
        text_color = (0, 255, 0) if self.lane_detected else (0, 0, 255)
        cv2.putText(
            roi, overlay_text, (50, 50),
            cv2.FONT_HERSHEY_SIMPLEX, 1, text_color, 2
        )

        result_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        self.image_pub.publish(result_msg)

    def process_loop(self):
        # Check if traffic signal has expired
        self.check_traffic_signal_timeout()
        
        # [START] Code added for parking signal timeout check (2025-08-25)
        self.check_parking_signal_timeout()
        # [END] Code added for parking signal timeout check (2025-08-25)
        
        # Check parking sign conditions first
        if self.should_handle_parking_maneuver():
            self.process_parking_maneuver_state()
        # Check traffic light conditions (only if signal is recent and active)
        elif self.should_handle_traffic_light():
            self.process_traffic_light_state()
        elif self.state == 'NORMAL':
            self.process_normal_state()
        elif self.state == 'AVOID_TURN':
            self.process_avoid_turn_state()
        elif self.state == 'AVOID_STRAIGHT':
            self.process_avoid_straight_state()
        elif self.state == 'RETURN_TURN':
            self.process_return_turn_state()
        elif self.state == 'TRAFFIC_STOP':
            self.process_traffic_stop_state()
        elif self.state == 'PARKING_MANEUVER':
            self.process_parking_maneuver_state()

    def check_traffic_signal_timeout(self):
        """Check if traffic signal has timed out and deactivate if necessary"""
        if (self.traffic_light_active and 
            self.traffic_last_update_time is not None):
            
            current_time = self.get_clock().now()
            time_diff = (current_time - self.traffic_last_update_time).nanoseconds / 1e9
            
            if time_diff > self.traffic_signal_timeout:
                self.get_logger().info('Traffic signal timeout - deactivating traffic light control')
                self.traffic_light_active = False
                self.traffic_type = None
                if self.state == 'TRAFFIC_STOP':
                    self.state = 'NORMAL'
                    self.publish_active(False)
    
    # [START] Modified parking duration check (2025-08-26)
    def check_parking_signal_timeout(self):
        """Check if minimum parking duration has passed before allowing deactivation"""
        if (self.parking_maneuver_active and 
            self.parking_start_time is not None):
            
            current_time = self.get_clock().now()
            time_since_start = (current_time - self.parking_start_time).nanoseconds / 1e9
            
            # Only allow deactivation after minimum duration AND if parking is completed
            if (time_since_start > self.parking_minimum_duration and 
                self.parking_phase in ['PARKED', 'DEPARKED', 'IDLE']):
                self.get_logger().info(f'Minimum parking duration ({self.parking_minimum_duration}s) completed - allowing normal return')
                # Note: Don't force deactivation here, let the parking completion logic handle it
            elif time_since_start < self.parking_minimum_duration:
                # Force parking to continue even if signal is lost
                if not self.parking_sign_detected:
                    self.get_logger().info(f'Parking signal lost but continuing task - {time_since_start:.1f}s / {self.parking_minimum_duration}s elapsed')
                    self.publish_active(False)
    # [END] Code added for parking signal timeout check (2025-08-25)

    def should_handle_traffic_light(self):
        """Check if we should handle traffic light based on recent signal and camera detection"""
        # Only handle traffic light if:
        # 1. We have a recent traffic signal
        # 2. Traffic light control is active
        # 3. We're not currently avoiding obstacles
        if (not self.traffic_light_active or 
            self.traffic_type is None or 
            self.state in ['AVOID_TURN', 'AVOID_STRAIGHT', 'RETURN_TURN']):
            return False
        
        return True
    
    # [START] Code added for parking maneuver condition check (2025-08-25)
    def should_handle_parking_maneuver(self):
        """Check if we should handle parking maneuver based on recent signal"""
        # Only handle parking if:
        # 1. We have a recent parking signal
        # 2. Parking maneuver is active
        # 3. We're not currently avoiding obstacles or handling traffic lights
        if (not self.parking_maneuver_active or 
            not self.parking_sign_detected or 
            self.state in ['AVOID_TURN', 'AVOID_STRAIGHT', 'RETURN_TURN', 'TRAFFIC_STOP']):
            return False
        
        return True
    # [END] Code added for parking maneuver condition check (2025-08-25)

    def process_traffic_light_state(self):
        """Handle traffic light behavior based on type"""
        if self.traffic_type == 'red':
            # Stop when red light is detected (regardless of distance)
            self.get_logger().info('Red light detected - Stopping')
            self.state = 'TRAFFIC_STOP'
            twist = Twist()  # Zero velocity
            self.publish_control_command(twist)
            self.publish_active(True)
            return
        
        elif self.traffic_type == 'yellow':
            # Go faster when yellow light detected
            self.get_logger().info('Yellow light detected - Speeding up')
            twist = Twist()
            twist.linear.x = self.speed * self.yellow_speed_multiplier
            twist.angular.z = 0.0
            self.publish_control_command(twist)
            self.publish_active(True)
            return
        
        elif self.traffic_type == 'green':
            # Continue normal operation for green light
            self.get_logger().info('Green light detected - Continue')
            # Let normal state handle the movement
            pass
        
        # If not stopping for red light, continue with normal operation
        if self.state != 'TRAFFIC_STOP':
            self.process_normal_state()

    def process_traffic_stop_state(self):
        """Handle stopping at red traffic light"""
        if self.traffic_type == 'green':
            self.get_logger().info('Green light detected - Resuming normal operation')
            self.state = 'NORMAL'
            self.publish_active(False)
        elif self.traffic_type == 'yellow':
            self.get_logger().info('Yellow light detected - Preparing to go')
            # Stay stopped but prepare to move
            twist = Twist()
            self.publish_control_command(twist)
            self.publish_active(True)
        else:
            # Stay stopped for red light
            twist = Twist()
            self.publish_control_command(twist)
            self.publish_active(True)

    def process_normal_state(self):
        # Check for obstacle avoidance first
        if self.lidar_points is not None and self.lidar_points.size > 0:
            danger_detected = False
            for pt in self.lidar_points:
                x, y = pt[0], pt[1]
                if 0 < y < self.danger_distance and abs(x) < (self.danger_width / 2):
                    danger_detected = True
                    break
            
            if danger_detected:
                self.get_logger().info('Danger zone intrusion detected.')
                if self.lane_state in [1, 3]:
                    self.original_theta = self.current_theta
                    if self.lane_state == 1:
                        self.turn_direction = 'right'
                        self.desired_theta = self.normalize_angle(
                            self.current_theta - math.radians(80)
                        )
                    elif self.lane_state == 3:
                        self.turn_direction = 'left'
                        self.desired_theta = self.normalize_angle(
                            self.current_theta + math.radians(80)
                        )
                    self.get_logger().info(f'Avoidance mode on: turning {self.turn_direction}.')
                    self.state = 'AVOID_TURN'
                else:
                    self.get_logger().info('lane_state value does not meet activation conditions.')

    def process_avoid_turn_state(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        error = self.normalize_angle(self.desired_theta - self.current_theta)
        
        # Calculate filtered derivative
        derivative = self.calculate_filtered_derivative(error, current_time)
        
        # Enhanced PD control with filtered derivative
        angular_z = self.turn_Kp * error + self.turn_Kd * derivative
        
        # Apply angular velocity and acceleration limits
        angular_z = self.apply_angular_limits(angular_z, dt)
        
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = angular_z
        self.publish_control_command(twist)
        self.publish_active(True)
        
        # Use hysteresis for state transition
        if self.check_turn_completion_with_hysteresis(error, 'AVOID_TURN'):
            self.get_logger().info('Avoidance turn completed. Moving straight.')
            self.error_history.clear()  # Reset filter history
            self.time_history.clear()
            self.state = 'AVOID_STRAIGHT'

    def process_avoid_straight_state(self):
        twist = Twist()
        twist.linear.x = self.speed
        
        # Small curvature maintenance for straight path
        # Use minimal angular correction based on front distance
        if self.front_distance < 1.0:  # If obstacle nearby
            # Apply small curvature to maintain safe distance
            if self.turn_direction == 'left':
                twist.angular.z = 0.05  # Small left turn
            elif self.turn_direction == 'right':
                twist.angular.z = -0.05  # Small right turn
            else:
                twist.angular.z = 0.0
        else:
            twist.angular.z = 0.0
        
        self.publish_control_command(twist)
        self.publish_active(True)
        
        if self.lane_detected:
            self.get_logger().info('Lane detected. Starting return turn.')
            self.desired_theta = self.normalize_angle(self.original_theta)
            # Reset filter state for return turn
            self.error_history.clear()
            self.time_history.clear()
            self.filtered_derivative = 0.0
            self.ewma_error = 0.0
            self.state = 'RETURN_TURN'

    def process_return_turn_state(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        error = self.normalize_angle(self.desired_theta - self.current_theta)
        
        # Calculate filtered derivative
        derivative = self.calculate_filtered_derivative(error, current_time)
        
        # Enhanced PD control with filtered derivative
        angular_z = self.turn_Kp * error + self.turn_Kd * derivative
        
        # Apply angular velocity and acceleration limits
        angular_z = self.apply_angular_limits(angular_z, dt)
        
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = angular_z
        self.publish_control_command(twist)
        self.publish_active(True)
        
        # Use hysteresis for state transition
        if self.check_turn_completion_with_hysteresis(error, 'RETURN_TURN'):
            self.get_logger().info('Return turn completed. Switching to normal driving.')
            # Reset all control states
            self.error_history.clear()
            self.time_history.clear()
            self.filtered_derivative = 0.0
            self.ewma_error = 0.0
            self.last_angular_velocity = 0.0
            self.state = 'NORMAL'
            self.publish_active(False)
            twist = Twist()
            self.publish_control_command(twist)
    
    # [START] Code added for parking maneuver state processing (2025-08-25) - PASS-BY PARKING
    def process_parking_maneuver_state(self):
        """Handle waypoint-based parking navigation"""
        
        # Set state to parking maneuver
        self.state = 'PARKING_MANEUVER'
        
        twist = Twist()
        current_time = self.get_clock().now()
        
        # Calculate distance to current waypoint
        if self.current_waypoint is not None:
            dx = self.current_waypoint[0] - self.current_pos_x
            dy = self.current_waypoint[1] - self.current_pos_y
            distance_to_waypoint = math.sqrt(dx**2 + dy**2)
            
            # Calculate angle to waypoint
            angle_to_waypoint = math.atan2(dy, dx)
            angle_error = angle_to_waypoint - self.current_theta
            
            # Normalize angle error to [-pi, pi]
            while angle_error > math.pi:
                angle_error -= 2 * math.pi
            while angle_error < -math.pi:
                angle_error += 2 * math.pi
            
            self.get_logger().info(f'Phase: {self.parking_phase}, Distance to waypoint: {distance_to_waypoint:.3f}m, Angle error: {angle_error:.3f}rad')
            
            # Check if waypoint is reached
            if distance_to_waypoint < self.waypoint_tolerance:
                self.advance_to_next_waypoint()
                return
            
            # Navigate toward current waypoint
            if self.parking_phase not in ['PARKED']:
                # Simple proportional control
                twist.linear.x = self.parking_speed
                twist.angular.z = angle_error * 2.0  # P control for heading
                
                # Limit angular velocity
                max_angular = 1.0
                twist.angular.z = max(-max_angular, min(max_angular, twist.angular.z))
        
        # Handle parking completion
        elif self.parking_phase == 'PARKED':
            if self.parking_stop_start_time is None:
                self.parking_stop_start_time = current_time
                self.get_logger().info(f'Arrived at parking space! Starting {self.parking_stop_duration}s stop.')
            
            time_elapsed = (current_time - self.parking_stop_start_time).nanoseconds / 1e9
            
            if time_elapsed >= self.parking_stop_duration:
                # Check if minimum parking duration has passed
                time_since_start = (current_time - self.parking_start_time).nanoseconds / 1e9
                
                if time_since_start >= self.parking_minimum_duration:
                    # Both parking stop and minimum duration completed - start departure
                    self.get_logger().info(f'Parking completed after {time_since_start:.1f}s - Starting departure to lane')
                    self.parking_phase = 'DEPARKED'
                    self.current_waypoint = self.return_to_lane_waypoint
                    self.parking_stop_start_time = None  # Clear parking stop timer
                    
                    self.get_logger().info(f'Starting departure to lane waypoint: ({self.return_to_lane_waypoint[0]:.3f}, {self.return_to_lane_waypoint[1]:.3f})')
                    # Don't return to normal yet - continue with waypoint navigation
                else:
                    # Parking stop finished but minimum duration not reached - continue parking
                    remaining_time = self.parking_minimum_duration - time_since_start
                    self.get_logger().info(f'Parking stop completed, but waiting for minimum duration - {remaining_time:.1f}s remaining')
                    twist = Twist()  # Stay stopped
            else:
                # Continue stopping
                remaining_time = self.parking_stop_duration - time_elapsed
                self.get_logger().info(f'Parking stop - {remaining_time:.1f}s remaining')
                twist = Twist()  # Stay stopped
        
        self.publish_control_command(twist)
        self.publish_active(True)
    
    def advance_to_next_waypoint(self):
        """Advance to the next waypoint in the parking sequence"""
        if self.parking_phase == 'TO_START':
            self.parking_phase = 'TO_MIDDLE'
            self.current_waypoint = self.middle_waypoint
            self.get_logger().info('Reached start waypoint. Moving to middle waypoint.')
            
        elif self.parking_phase == 'TO_MIDDLE':
            self.parking_phase = 'TO_PARKING'
            self.current_waypoint = self.selected_parking_space
            space_num = 1 if self.selected_parking_space == self.parking_space_1 else 2
            self.get_logger().info(f'Reached middle waypoint. Moving to parking space {space_num}.')
            
        elif self.parking_phase == 'TO_PARKING':
            self.parking_phase = 'PARKED'
            self.current_waypoint = None
            space_num = 1 if self.selected_parking_space == self.parking_space_1 else 2
            self.get_logger().info(f'Reached parking space {space_num}!')
            
        elif self.parking_phase == 'DEPARKED':
            # Reached lane return waypoint - complete parking sequence
            self.get_logger().info('Reached lane return waypoint - Parking sequence completed!')
            self.get_logger().info('Returning to NORMAL lane following mode')
            
            # Reset all parking states and return to normal
            self.parking_maneuver_active = False
            self.parking_sign_detected = False
            self.parking_phase = 'IDLE'
            self.current_waypoint = None
            self.selected_parking_space = None
            self.parking_stop_start_time = None
            self.parking_start_time = None
            self.state = 'NORMAL'
            self.publish_active(False)
    # [END] Waypoint-based parking maneuver state processing (2025-08-26)

    def publish_active(self, active: bool):
        bool_msg = Bool()
        bool_msg.data = active
        self.avoid_active_pub.publish(bool_msg)

    def visualization(self, points):
        img_vis = np.zeros((500, 500, 3), dtype=np.uint8)
        scale = 400.0  # 1 m = 400 pixels
        center = (img_vis.shape[1] // 2, img_vis.shape[0] // 2)
        robot_width_m = 0.12
        robot_height_m = 0.12
        robot_width_px = int(robot_width_m * scale)
        robot_height_px = int(robot_height_m * scale)
        tb_top_left = (center[0] - robot_width_px // 2, center[1] - robot_height_px // 2)
        tb_bottom_right = (center[0] + robot_width_px // 2, center[1] + robot_height_px // 2)
        cv2.rectangle(img_vis, tb_top_left, tb_bottom_right, (255, 0, 0), 2)
        cv2.putText(img_vis, 'TurtleBot', (tb_top_left[0], tb_top_left[1] - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Danger zone
        danger_height = self.danger_distance - (robot_height_m / 2)
        danger_width = robot_width_m
        dz_top_left = (
            center[0] - int(danger_width * scale / 2), tb_top_left[1] - int(danger_height * scale)
        )
        dz_bottom_right = (
            center[0] + int(danger_width * scale / 2), tb_top_left[1]
        )
        overlay = img_vis.copy()
        cv2.rectangle(overlay, dz_top_left, dz_bottom_right, (0, 0, 255), -1)
        alpha = 0.3
        img_vis = cv2.addWeighted(overlay, alpha, img_vis, 1 - alpha, 0)
        
        # Traffic light detection zone
        tl_height = self.traffic_light_detected_distance - (robot_height_m / 2)
        tl_width = robot_width_m * 2
        tl_top_left = (
            center[0] - int(tl_width * scale / 2), tb_top_left[1] - int(tl_height * scale)
        )
        tl_bottom_right = (
            center[0] + int(tl_width * scale / 2), tb_top_left[1]
        )
        overlay2 = img_vis.copy()
        cv2.rectangle(overlay2, tl_top_left, tl_bottom_right, (255, 255, 0), -1)
        alpha2 = 0.2
        img_vis = cv2.addWeighted(overlay2, alpha2, img_vis, 1 - alpha2, 0)
        
        if points is not None:
            for pt in points:
                x = int(pt[0] * scale + center[0])
                y = int(-pt[1] * scale + center[1])
                cv2.circle(img_vis, (x, y), 2, (0, 255, 0), -1)
        
        # Display current state and traffic light info
        state_text = f'State: {self.state}'
        traffic_text = f'Traffic: {self.traffic_type if self.traffic_type else "None"}'
        traffic_active_text = f'TL Active: {self.traffic_light_active}'
        distance_text = f'Front Dist: {self.front_distance:.2f}m'
        # [START] Waypoint-based parking status display (2025-08-26)
        parking_text = f'Parking: {self.parking_phase if self.parking_maneuver_active else "Inactive"}'
        
        if self.current_waypoint is not None:
            parking_waypoint_text = f'Target: ({self.current_waypoint[0]:.2f}, {self.current_waypoint[1]:.2f})'
            dx = self.current_waypoint[0] - self.current_pos_x
            dy = self.current_waypoint[1] - self.current_pos_y
            distance_to_waypoint = math.sqrt(dx**2 + dy**2)
            parking_distance_text = f'Dist to WP: {distance_to_waypoint:.2f}m'
        else:
            parking_waypoint_text = f'Target: None'
            parking_distance_text = f'Dist to WP: -'
        
        if self.selected_parking_space is not None:
            space_num = 1 if self.selected_parking_space == self.parking_space_1 else 2
            parking_space_text = f'Selected Space: {space_num}'
        else:
            parking_space_text = f'Selected Space: None'
            
        # Show parking duration
        if self.parking_start_time is not None:
            current_time = self.get_clock().now()
            elapsed_time = (current_time - self.parking_start_time).nanoseconds / 1e9
            parking_time_text = f'Parking Time: {elapsed_time:.1f}s/{self.parking_minimum_duration:.0f}s'
        else:
            parking_time_text = f'Parking Time: -'
        # [END] Waypoint-based parking status display (2025-08-26)
        
        cv2.putText(img_vis, state_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(img_vis, traffic_text, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(img_vis, traffic_active_text, (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(img_vis, distance_text, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        # [START] Waypoint-based parking status display (2025-08-26)
        cv2.putText(img_vis, parking_text, (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(img_vis, parking_waypoint_text, (10, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(img_vis, parking_distance_text, (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(img_vis, parking_space_text, (10, 170), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(img_vis, parking_time_text, (10, 190), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        # [END] Waypoint-based parking status display (2025-08-26)
        
        cv2.imshow('Cluster Points & Danger Zone', img_vis)
        cv2.waitKey(1)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def calculate_filtered_derivative(self, error, current_time):
        """
        Calculate derivative using Savitzky-Golay filter with real-time dt
        """
        # Add current error and time to history
        self.error_history.append(error)
        self.time_history.append(current_time)
        
        if len(self.error_history) < 3:  # Need at least 3 points
            return 0.0
        
        # Convert time to seconds for calculation
        times = np.array([(t - self.time_history[0]).nanoseconds / 1e9 
                         for t in self.time_history])
        errors = np.array(self.error_history)
        
        if len(errors) >= self.sg_window_size:
            # Use Savitzky-Golay filter
            try:
                # Calculate derivative using S-G filter
                sg_deriv = signal.savgol_filter(errors, self.sg_window_size, 
                                              self.sg_poly_order, deriv=1, 
                                              delta=np.mean(np.diff(times)))
                raw_derivative = sg_deriv[-1]  # Take the latest derivative
            except:
                # Fallback to simple difference if S-G fails
                dt = times[-1] - times[-2] if len(times) > 1 else 0.1
                raw_derivative = (errors[-1] - errors[-2]) / dt if dt > 0 else 0.0
        else:
            # Use simple difference for initial samples
            dt = times[-1] - times[-2] if len(times) > 1 else 0.1
            raw_derivative = (errors[-1] - errors[-2]) / dt if dt > 0 else 0.0
        
        # Apply low-pass filter to smooth the derivative
        self.filtered_derivative = (self.lpf_alpha * raw_derivative + 
                                  (1 - self.lpf_alpha) * self.filtered_derivative)
        
        return self.filtered_derivative
    
    def apply_angular_limits(self, desired_angular_vel, dt):
        """
        Apply velocity and acceleration limits to angular velocity
        """
        # Apply velocity saturation
        limited_vel = np.clip(desired_angular_vel, 
                             -self.max_angular_velocity, 
                             self.max_angular_velocity)
        
        # Apply acceleration (rate) limits
        if dt > 0:
            max_change = self.max_angular_acceleration * dt
            vel_change = limited_vel - self.last_angular_velocity
            vel_change = np.clip(vel_change, -max_change, max_change)
            final_vel = self.last_angular_velocity + vel_change
        else:
            final_vel = limited_vel
        
        self.last_angular_velocity = final_vel
        return final_vel
    
    def update_ewma_error(self, error):
        """
        Update exponentially weighted moving average of error
        """
        self.ewma_error = (self.ewma_alpha * abs(error) + 
                          (1 - self.ewma_alpha) * self.ewma_error)
        return self.ewma_error
    
    def check_turn_completion_with_hysteresis(self, error, current_state):
        """
        Check turn completion using hysteresis to prevent chattering
        """
        ewma_error = self.update_ewma_error(error)
        
        if current_state in ['AVOID_TURN', 'RETURN_TURN']:
            # Use stricter threshold to complete turn (prevent early exit)
            return ewma_error < self.turn_threshold_enter
        else:
            # Use looser threshold to start turn (prevent late entry)
            return ewma_error < self.turn_threshold_exit


def main(args=None):
    rclpy.init(args=args)
    node = AvoidConstruction()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()