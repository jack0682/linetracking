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
        
        # Subscribe to navigation status
        self.nav_status_sub = self.create_subscription(
            Bool,
            '/navigation_active',
            self.nav_status_callback,
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
        
        # Navigation state
        self.navigation_active = False
        
        # Additional publisher for dummy commands when navigation is active
        self.dummy_cmd_pub = self.create_publisher(
            Twist,
            '/dummy_lane_cmd',
            10
        )

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
        
        # [START] Code added for parking sign parameters (2025-08-25)
        # Parking sign parameters
        self.parking_sign_detected = False
        self.parking_last_update_time = None
        self.parking_signal_timeout = 3.0  # Timeout for parking signal (seconds)
        self.parking_maneuver_active = False
        self.parking_maneuver_distance = 0.3  # Distance to move forward before parking maneuver
        
        # Parking sign center position
        self.parking_center_x = 0.0  # Normalized X position (-1 to 1)
        self.parking_center_y = 0.0  # Normalized Y position (-1 to 1)
        self.parking_confidence = 0.0  # Detection confidence
        
        # Parking approach control parameters (IMPROVED FOR COLLISION AVOIDANCE)
        self.parking_approach_speed = 0.01   # Even slower approach speed (was 0.015)
        self.parking_angle_threshold = 0.1   # Threshold for center alignment
        self.parking_target_distance = 0.6   # Target distance from parking sign (meters)
        self.parking_stop_duration = 5.0     # Duration to stop at parking spot (seconds)
        
        # Enhanced distance tracking for PASS-BY parking
        self.parking_closest_distance = float('inf')  # Track closest distance to parking sign
        self.parking_stop_start_time = None          # Time when parking stop started
        self.parking_state = 'APPROACHING'           # 'APPROACHING', 'PASSING', 'STOPPING', 'STOPPED'
        
        # Lane-keeping parking parameters (stay within lane)
        self.parking_lateral_offset = 0.15          # Small lateral offset within lane (15cm)
        self.parking_forward_distance = 0.6         # Distance to travel forward past sign center
        self.parking_initial_front_distance = None  # Store initial front distance when sign detected
        self.parking_passed_sign_center = False     # Flag to track if we've passed sign center
        self.parking_pass_start_position = None     # Position when starting to pass
        
        # Safety and control parameters for lane-keeping parking
        self.parking_emergency_stop_distance = 0.12  # Emergency stop if too close (12cm)
        self.parking_approach_speed_normal = 0.02    # Slower speed to stay in lane
        self.parking_approach_speed_passing = 0.015  # Speed when passing sign
        self.parking_max_lateral_angular = 0.15     # Max angular velocity for lane-keeping (reduced)
        self.parking_distance_samples = deque(maxlen=5)  # Smooth distance measurements
        self.parking_stable_count = 0                # Count for stable distance readings
        # [END] Code added for parking sign parameters (2025-08-25)

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
    
    def nav_status_callback(self, msg):
        """Navigation status callback - switches control mode"""
        prev_state = self.navigation_active
        self.navigation_active = msg.data
        
        if prev_state != self.navigation_active:
            if self.navigation_active:
                self.get_logger().info('Navigation ACTIVE - Lane control DISABLED')
            else:
                self.get_logger().info('Navigation INACTIVE - Lane control ENABLED')
    
    def publish_control_command(self, twist):
        """
        Publish control command - routes to appropriate topic based on navigation state
        
        Modified: 2025-08-26 11:32 - Added navigation state-based routing
        - When navigation_active=True: commands sent to dummy topic (disabled)
        - When navigation_active=False: commands sent to normal lane control topic
        This prevents lane following from interfering with autonomous navigation
        """
        if self.navigation_active:
            # Navigation is active - send to dummy topic (lane control disabled)
            self.dummy_cmd_pub.publish(twist)
        else:
            # Navigation is not active - send to normal lane control
            self.publish_control_command(twist)  # Modified: 2025-08-26 11:32 - Route via navigation-aware control
    
    # [START] Code added for parking sign callback (2025-08-25)
    def parking_sign_callback(self, msg):
        """Handle parking sign detection signal"""
        if msg.data == 1:  # Parking sign detected (assuming value 1 means parking)
            self.parking_sign_detected = True
            self.parking_last_update_time = self.get_clock().now()
            if not self.parking_maneuver_active:
                self.parking_maneuver_active = True
                self.parking_state = 'APPROACHING'
                self.parking_closest_distance = float('inf')
                self.parking_stop_start_time = None
                self.get_logger().info('Parking sign detected - Starting slow approach')
    
    def parking_center_callback(self, msg):
        """Handle parking sign center position"""
        self.parking_center_x = msg.x  # -1 (left) to 1 (right)
        self.parking_center_y = msg.y  # -1 (top) to 1 (bottom)
        self.parking_confidence = msg.z  # Detection confidence
        self.get_logger().info(f'Parking center updated: ({self.parking_center_x:.3f}, {self.parking_center_y:.3f})')
    # [END] Code added for parking sign callback (2025-08-25)

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
    
    # [START] Code added for parking signal timeout check (2025-08-25)
    def check_parking_signal_timeout(self):
        """Check if parking signal has timed out and deactivate if necessary"""
        if (self.parking_maneuver_active and 
            self.parking_last_update_time is not None):
            
            current_time = self.get_clock().now()
            time_diff = (current_time - self.parking_last_update_time).nanoseconds / 1e9
            
            if time_diff > self.parking_signal_timeout:
                self.get_logger().info('Parking signal timeout - deactivating parking maneuver')
                self.parking_maneuver_active = False
                self.parking_sign_detected = False
                if self.state == 'PARKING_MANEUVER':
                    self.state = 'NORMAL'
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
            self.publish_control_command(twist)  # Modified: 2025-08-26 11:32 - Route via navigation-aware control
            self.publish_active(True)
            return
        
        elif self.traffic_type == 'yellow':
            # Go faster when yellow light detected
            self.get_logger().info('Yellow light detected - Speeding up')
            twist = Twist()
            twist.linear.x = self.speed * self.yellow_speed_multiplier
            twist.angular.z = 0.0
            self.publish_control_command(twist)  # Modified: 2025-08-26 11:32 - Route via navigation-aware control
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
            self.publish_control_command(twist)  # Modified: 2025-08-26 11:32 - Route via navigation-aware control
            self.publish_active(True)
        else:
            # Stay stopped for red light
            twist = Twist()
            self.publish_control_command(twist)  # Modified: 2025-08-26 11:32 - Route via navigation-aware control
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
        self.publish_control_command(twist)  # Modified: 2025-08-26 11:32 - Route via navigation-aware control
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
        
        self.publish_control_command(twist)  # Modified: 2025-08-26 11:32 - Route via navigation-aware control
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
        self.publish_control_command(twist)  # Modified: 2025-08-26 11:32 - Route via navigation-aware control
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
            self.publish_control_command(twist)  # Modified: 2025-08-26 11:32 - Route via navigation-aware control
    
    # [START] Code added for parking maneuver state processing (2025-08-25) - PASS-BY PARKING
    def process_parking_maneuver_state(self):
        """Handle parking maneuver behavior - PASS-BY parking (drive past sign and stop beside it)"""
        
        # Set state to parking maneuver
        self.state = 'PARKING_MANEUVER'
        
        twist = Twist()
        current_time = self.get_clock().now()
        
        # Smooth distance measurement using moving average
        self.parking_distance_samples.append(self.front_distance)
        if len(self.parking_distance_samples) > 0:
            smooth_distance = sum(self.parking_distance_samples) / len(self.parking_distance_samples)
        else:
            smooth_distance = self.front_distance
            
        # CRITICAL SAFETY CHECK - Emergency stop if too close
        if smooth_distance < self.parking_emergency_stop_distance:
            self.get_logger().error(f'EMERGENCY STOP: Too close to obstacle ({smooth_distance:.3f}m < {self.parking_emergency_stop_distance}m)')
            self.parking_state = 'STOPPING'
            self.parking_stop_start_time = current_time
            twist = Twist()  # Full stop
            self.publish_control_command(twist)  # Modified: 2025-08-26 11:32 - Route via navigation-aware control
            self.publish_active(True)
            return
        
        if self.parking_state == 'APPROACHING':
            # Store initial distance when first detecting the sign
            if self.parking_initial_front_distance is None:
                self.parking_initial_front_distance = smooth_distance
                self.get_logger().info(f'Starting lane-keeping parking approach. Initial distance: {smooth_distance:.3f}m')
            
            # Track the closest distance to parking sign
            if smooth_distance < self.parking_closest_distance:
                self.parking_closest_distance = smooth_distance
                self.parking_stable_count = 0
                self.get_logger().info(f'Closest distance updated: {self.parking_closest_distance:.3f}m')
            else:
                self.parking_stable_count += 1
            
            # Determine which side the sign is on and make SMALL adjustment to move slightly away
            sign_side = 'right' if self.parking_center_x > 0 else 'left'
            
            # Make small lateral adjustment to avoid direct collision while staying in lane
            if sign_side == 'right':
                # Sign is on right, move slightly left within lane
                target_lateral_adjustment = -self.parking_lateral_offset  # Small left turn
            else:
                # Sign is on left, move slightly right within lane  
                target_lateral_adjustment = self.parking_lateral_offset   # Small right turn
            
            # Very gentle steering to stay within lane boundaries
            angular_z = target_lateral_adjustment * 0.8  # Gentle steering within lane
            angular_z = max(-self.parking_max_lateral_angular, min(self.parking_max_lateral_angular, angular_z))
            
            # Check if we've passed the sign center (parking_center_x changes from one side to other)
            # or if we're at a reasonable distance past the closest point
            if (self.parking_closest_distance < 1.0 and 
                smooth_distance > self.parking_closest_distance + 0.15 and
                self.parking_stable_count >= 3):
                
                self.parking_state = 'PASSING'
                self.parking_pass_start_position = (self.current_pos_x, self.current_pos_y)
                self.get_logger().info(f'Started passing sign center, moving to parking position. Distance: {smooth_distance:.3f}m')
            
            # Continue approaching with small lateral adjustment
            self.get_logger().info(f'Approaching (lane-keeping) - Sign on {sign_side}, Distance: {smooth_distance:.3f}m, Angular: {angular_z:.3f}')
            twist.linear.x = self.parking_approach_speed_normal
            twist.angular.z = angular_z
            
        elif self.parking_state == 'PASSING':
            # Continue driving forward within lane to position beside the sign
            if self.parking_pass_start_position is not None:
                # Calculate distance traveled since starting to pass
                dx = self.current_pos_x - self.parking_pass_start_position[0]
                dy = self.current_pos_y - self.parking_pass_start_position[1]
                distance_traveled = (dx**2 + dy**2)**0.5
                
                # Check if we've traveled enough distance to be beside the sign
                if distance_traveled >= self.parking_forward_distance:
                    self.parking_state = 'STOPPING'
                    self.parking_stop_start_time = current_time
                    self.get_logger().info(f'Reached parking position beside sign ({distance_traveled:.3f}m forward). Starting 5-second stop.')
                    twist = Twist()  # Stop immediately
                    self.publish_control_command(twist)  # Modified: 2025-08-26 11:32 - Route via navigation-aware control
                    self.publish_active(True)
                    return
            
            # Continue moving forward slowly while staying in lane
            # Apply minimal steering correction to stay centered in lane
            lane_correction = 0.0
            if abs(self.parking_center_x) > 0.8:  # Only correct if sign is very far to one side
                lane_correction = -self.parking_center_x * 0.1  # Very gentle correction
                lane_correction = max(-0.1, min(0.1, lane_correction))  # Limit correction
            
            self.get_logger().info(f'Moving forward to parking position - Distance traveled: {distance_traveled:.3f}m/{self.parking_forward_distance:.3f}m')
            twist.linear.x = self.parking_approach_speed_passing
            twist.angular.z = lane_correction  # Very small lane-keeping adjustment
        
        elif self.parking_state == 'STOPPING':
            # Stay stopped for the specified duration
            time_elapsed = (current_time - self.parking_stop_start_time).nanoseconds / 1e9
            
            if time_elapsed >= self.parking_stop_duration:
                # Finished parking, return to normal
                self.get_logger().info('Lane-keeping parking completed - Resuming normal operation')
                self.parking_maneuver_active = False
                self.parking_sign_detected = False
                self.parking_closest_distance = float('inf')
                self.parking_stable_count = 0
                self.parking_distance_samples.clear()
                self.parking_initial_front_distance = None
                self.parking_passed_sign_center = False
                self.parking_pass_start_position = None
                self.state = 'NORMAL'
                self.publish_active(False)
                twist = Twist()  # Ensure stopped
            else:
                # Continue stopping
                remaining_time = self.parking_stop_duration - time_elapsed
                self.get_logger().info(f'Parking stop beside sign (in lane) - {remaining_time:.1f}s remaining')
                twist = Twist()  # Stay stopped
        
        self.publish_control_command(twist)  # Modified: 2025-08-26 11:32 - Route via navigation-aware control
        self.publish_active(True)
    # [END] Code added for parking maneuver state processing (2025-08-25)

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
        # [START] Code added for parking status display (2025-08-25)
        # [UPDATED] Status display for lane-keeping parking (2025-08-25 18:14)
        parking_text = f'Parking: {self.parking_state if self.parking_maneuver_active else "Inactive"}'
        parking_center_text = f'P-Center: ({self.parking_center_x:.2f}, {self.parking_center_y:.2f})'
        parking_distance_text = f'P-Closest: {self.parking_closest_distance:.2f}m'
        # [END] Code added for parking status display (2025-08-25)
        # [END] Lane-keeping parking status display (2025-08-25 18:14)
        
        cv2.putText(img_vis, state_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(img_vis, traffic_text, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(img_vis, traffic_active_text, (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(img_vis, distance_text, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        # [START] Code added for parking status display (2025-08-25)
        # [UPDATED] Status display for lane-keeping parking (2025-08-25 18:14)
        cv2.putText(img_vis, parking_text, (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(img_vis, parking_center_text, (10, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(img_vis, parking_distance_text, (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        # [END] Code added for parking status display (2025-08-25)
        # [END] Lane-keeping parking status display (2025-08-25 18:14)
        
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