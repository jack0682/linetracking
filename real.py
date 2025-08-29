"""스플라인 버전"""

#!/usr/bin/env python3
#
# Copyright 2018 ROBOTIS CO., LTD.
#
# Hybrid EWMA-Spline Lane Control for TurtleBot3
# Combines efficiency of EWMA with accuracy of Spline interpolation

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from std_msgs.msg import UInt8

import csv
import os
import time
import threading
from datetime import datetime
import numpy as np
from collections import deque

# Optional spline with fallback
try:
    from scipy.interpolate import CubicSpline
    _SCIPY_OK = True
except ImportError:
    CubicSpline = None
    _SCIPY_OK = False


class HybridControlLogger:
    """Optimized logger for hybrid control method"""
    def __init__(self, log_dir="lane_control_logs", buffer_size=50):
        self.log_dir = log_dir
        os.makedirs(log_dir, exist_ok=True)
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_file = os.path.join(log_dir, f"hybrid_ewma_spline_{timestamp}.csv")
        
        self.csv_headers = [
            'timestamp', 'relative_time', 'method', 'lane_center', 'error_raw',
            'ewma_filtered', 'spline_filtered', 'final_filtered', 
            'ewma_derivative', 'spline_derivative', 'final_derivative',
            'ewma_weight', 'spline_weight', 'confidence_score',
            'angular_z_before_sat', 'angular_z_output', 'linear_x_output',
            'computation_time_ms', 'sample_interval_ms', 'control_effort',
            'lane_state', 'max_vel_setting', 'spline_active', 'spline_samples'
        ]
        
        # Initialize CSV file
        with open(self.csv_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(self.csv_headers)
        
        # Buffering system
        self.buffer_size = buffer_size
        self.data_buffer = []
        self.lock = threading.Lock()
        self.start_time = time.time()
        self.prev_angular_z = 0.0
        self.prev_timestamp = None
        
        print(f"[LOGGER] Hybrid EWMA-Spline logging to: {self.csv_file}")
        
    def log_control_data(self, lane_center, error_raw, ewma_filtered, spline_filtered,
                        final_filtered, ewma_derivative, spline_derivative, final_derivative,
                        ewma_weight, spline_weight, confidence_score, angular_z_before_sat,
                        angular_z_output, linear_x_output, computation_time_ms,
                        lane_state=None, max_vel=None, spline_active=False, spline_samples=0):
        
        current_time = time.time()
        relative_time = current_time - self.start_time
        
        # Calculate control effort
        control_effort = abs(angular_z_output - self.prev_angular_z)
        self.prev_angular_z = angular_z_output
        
        # Calculate sample interval
        sample_interval_ms = 0.0
        if self.prev_timestamp is not None:
            sample_interval_ms = (current_time - self.prev_timestamp) * 1000
        self.prev_timestamp = current_time
        
        row_data = [
            current_time, relative_time, 'hybrid_ewma_spline', lane_center, error_raw,
            ewma_filtered, spline_filtered if spline_filtered is not None else '',
            final_filtered, ewma_derivative, spline_derivative if spline_derivative is not None else '',
            final_derivative, ewma_weight, spline_weight, confidence_score,
            angular_z_before_sat, angular_z_output, linear_x_output,
            computation_time_ms, sample_interval_ms, control_effort,
            lane_state if lane_state is not None else '', max_vel if max_vel is not None else '',
            spline_active, spline_samples
        ]
        
        # Buffered writing
        with self.lock:
            self.data_buffer.append(row_data)
            if len(self.data_buffer) >= self.buffer_size:
                self._flush_buffer()
    
    def _flush_buffer(self):
        if self.data_buffer:
            with open(self.csv_file, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerows(self.data_buffer)
            self.data_buffer.clear()
    
    def __del__(self):
        with self.lock:
            self._flush_buffer()


import time

class EWMAEstimator:
    """Exponentially Weighted Moving Average estimator with robust dt handling."""
    def __init__(self, alpha: float = 0.3, dt_nom: float = 0.033,
                 dt_min: float = 1e-4, dt_max: float = 0.5):
        assert 0.0 < alpha < 1.0, "alpha must be in (0,1)"
        self.alpha = alpha
        self.dt_nom = dt_nom
        self.dt_min = dt_min
        self.dt_max = dt_max

        self.smoothed_error = None
        self.prev_smoothed_error = None
        self.derivative = 0.0
        self._last_time = None  # last timestamp (perf_counter or external)

    def reset(self):
        self.smoothed_error = None
        self.prev_smoothed_error = None
        self.derivative = 0.0
        self._last_time = None

    def update(self, error: float, t_now: float | None = None):
        """
        Update EWMA with new error sample.
        Args:
            error: current raw error
            t_now: current time in seconds (optional). If None, uses time.perf_counter().
        Returns:
            (smoothed_error, derivative) where derivative is d(smoothed_error)/dt
        """
        # timestamp
        now = time.perf_counter() if t_now is None else float(t_now)

        # first sample (warm-up)
        if self.smoothed_error is None or self._last_time is None:
            self.smoothed_error = error
            self.prev_smoothed_error = error
            self.derivative = 0.0
            self._last_time = now
            return self.smoothed_error, self.derivative

        # compute dt robustly
        dt = now - self._last_time
        self._last_time = now
        # clamp dt; if out-of-bound, fallback to dt_nom
        if not (self.dt_min <= dt <= self.dt_max):
            dt = self.dt_nom

        # EWMA smoothing
        self.prev_smoothed_error = self.smoothed_error
        self.smoothed_error = self.alpha * error + (1.0 - self.alpha) * self.prev_smoothed_error

        # derivative of the *smoothed* signal
        self.derivative = (self.smoothed_error - self.prev_smoothed_error) / dt

        return self.smoothed_error, self.derivative


class OptimizedSplineEstimator:
    """Optimized spline estimator for hybrid use"""
    def __init__(self, maxlen=15, min_samples=5, refit_interval=0.05):
        self.maxlen = maxlen
        self.min_samples = min_samples
        self.refit_interval = refit_interval
        
        self.t_buf = deque(maxlen=maxlen)
        self.e_buf = deque(maxlen=maxlen)
        self.spline = None
        
        # Performance optimization
        self.last_fit_time = 0.0
        self.cache_result = None
        self.cache_time = 0.0
        self.cache_validity = 0.01  # 10ms cache
        
    def add(self, t_sec, error):
        self.t_buf.append(t_sec)
        self.e_buf.append(error)
        
        # Selective refitting
        current_time = time.time()
        if (len(self.t_buf) >= self.min_samples and 
            (current_time - self.last_fit_time) > self.refit_interval):
            self._fit_spline()
            self.last_fit_time = current_time
    
    def _fit_spline(self):
        try:
            if len(self.t_buf) < self.min_samples:
                return
                
            t_array = np.array(self.t_buf)
            e_array = np.array(self.e_buf)
            
            # Sort and remove duplicates
            sort_idx = np.argsort(t_array)
            t_sorted = t_array[sort_idx]
            e_sorted = e_array[sort_idx]
            
            # Remove duplicate time points
            dt = np.diff(t_sorted)
            unique_mask = np.concatenate(([True], dt > 1e-6))
            t_unique = t_sorted[unique_mask]
            e_unique = e_sorted[unique_mask]
            
            if len(t_unique) >= self.min_samples and (t_unique[-1] - t_unique[0]) > 1e-3:
                self.spline = CubicSpline(t_unique, e_unique, bc_type='natural')
            else:
                self.spline = None
                
        except Exception:
            self.spline = None
    
    def eval(self, t_now):
        if self.spline is None:
            return None, None
            
        # Check cache
        current_time = time.time()
        if (self.cache_result is not None and 
            abs(current_time - self.cache_time) < self.cache_validity):
            return self.cache_result
        
        try:
            # Clamp to avoid extrapolation
            t0, t1 = self.spline.x[0], self.spline.x[-1]
            tt = max(t0, min(t1, t_now))
            
            e_hat = float(self.spline(tt, 0))
            de_dt = float(self.spline(tt, 1))
            
            result = (e_hat, de_dt)
            self.cache_result = result
            self.cache_time = current_time
            
            return result
            
        except Exception:
            return None, None


class HybridEWMASplineEstimator:
    """Hybrid estimator combining EWMA and Spline methods"""
    def __init__(self, ewma_alpha=0.25, dt_nom=0.033):
        self.ewma = EWMAEstimator(alpha=ewma_alpha, dt_nom=dt_nom)
        
        if _SCIPY_OK:
            self.spline = OptimizedSplineEstimator(maxlen=15, min_samples=5)
            self.spline_available = True
        else:
            self.spline = None
            self.spline_available = False
        
        # Adaptive weighting parameters
        self.base_ewma_weight = 0.7  # Higher weight for reliable EWMA
        self.base_spline_weight = 0.3
        self.confidence_history = deque(maxlen=10)
        
    def update(self, error, current_time):
        # Always get EWMA estimate
        ewma_filtered, ewma_derivative = self.ewma.update(error)
        
        # Try to get spline estimate
        spline_filtered = None
        spline_derivative = None
        spline_active = False
        spline_samples = 0
        
        if self.spline_available:
            self.spline.add(current_time, error)
            spline_samples = len(self.spline.t_buf)
            
            spline_result = self.spline.eval(current_time)
            if spline_result[0] is not None:
                spline_filtered, spline_de_dt = spline_result
                spline_derivative = spline_de_dt * self.ewma.dt_nom  # Scale to match EWMA
                spline_active = True
        
        # Adaptive weight calculation
        ewma_weight, spline_weight, confidence = self._calculate_weights(
            ewma_filtered, ewma_derivative, spline_filtered, spline_derivative, spline_active
        )
        
        # Combine estimates
        if spline_active and spline_filtered is not None:
            final_filtered = (ewma_weight * ewma_filtered + 
                            spline_weight * spline_filtered)
            final_derivative = (ewma_weight * ewma_derivative + 
                              spline_weight * spline_derivative)
        else:
            final_filtered = ewma_filtered
            final_derivative = ewma_derivative
            spline_weight = 0.0
            ewma_weight = 1.0
        
        return {
            'final_filtered': final_filtered,
            'final_derivative': final_derivative,
            'ewma_filtered': ewma_filtered,
            'ewma_derivative': ewma_derivative,
            'spline_filtered': spline_filtered,
            'spline_derivative': spline_derivative,
            'ewma_weight': ewma_weight,
            'spline_weight': spline_weight,
            'confidence': confidence,
            'spline_active': spline_active,
            'spline_samples': spline_samples
        }
    
    def _calculate_weights(self, ewma_filtered, ewma_derivative, 
                         spline_filtered, spline_derivative, spline_active):
        """Calculate adaptive weights based on estimate confidence"""
        if not spline_active or spline_filtered is None:
            return 1.0, 0.0, 0.5
        
        # Calculate confidence based on estimate agreement
        error_diff = abs(ewma_filtered - spline_filtered)
        derivative_diff = abs(ewma_derivative - spline_derivative)
        
        # Normalize differences (assuming typical error range of ±100 pixels)
        normalized_error_diff = min(error_diff / 50.0, 1.0)
        normalized_deriv_diff = min(derivative_diff / 10.0, 1.0)
        
        # Confidence inversely related to disagreement
        agreement_score = 1.0 - 0.5 * (normalized_error_diff + normalized_deriv_diff)
        
        # Update confidence history
        self.confidence_history.append(agreement_score)
        avg_confidence = np.mean(self.confidence_history)
        
        # Adaptive weighting: higher spline weight when estimates agree
        if avg_confidence > 0.7:  # High agreement
            ewma_weight = 0.4
            spline_weight = 0.6
        elif avg_confidence > 0.5:  # Moderate agreement
            ewma_weight = 0.6
            spline_weight = 0.4
        else:  # Low agreement - trust EWMA more
            ewma_weight = 0.8
            spline_weight = 0.2
        
        return ewma_weight, spline_weight, avg_confidence


class TimingContext:
    def __init__(self):
        self.start = None
        self.end = None
    
    def __enter__(self):
        self.start = time.perf_counter()
        return self
    
    def __exit__(self, *args):
        self.end = time.perf_counter()
    
    def get_ms(self):
        return (self.end - self.start) * 1000 if self.start and self.end else 0.0


class ControlLane(Node):

    def __init__(self):
        super().__init__('control_lane_hybrid')

        # ROS subscriptions
        self.sub_lane = self.create_subscription(Float64, '/control/lane', self.callback_follow_lane, 1)
        self.sub_max_vel = self.create_subscription(Float64, '/control/max_vel', self.callback_get_max_vel, 1)
        self.sub_avoid_cmd = self.create_subscription(Twist, '/avoid_control', self.callback_avoid_cmd, 1)
        self.sub_avoid_active = self.create_subscription(Bool, '/avoid_active', self.callback_avoid_active, 1)
        self.sub_lane_state = self.create_subscription(UInt8, '/detect/lane_state', self.callback_lane_state, 1)

        self.pub_cmd_vel = self.create_publisher(Twist, '/control/cmd_vel', 1)

        # Control parameters
        self.last_error = 0.0
        self.MAX_VEL = 0.1
        self.avoid_active = False
        self.avoid_twist = Twist()
        self.lane_state = None
        self.center_px = 500.0

        # Initialize hybrid estimator
        self.hybrid_estimator = HybridEWMASplineEstimator(ewma_alpha=0.25, dt_nom=0.033)

        # PD control gains (optimized for hybrid method)
        self.Kp = 0.0025
        self.Kd = 0.005  # Slightly reduced due to better derivative estimation

        # Initialize logger
        self.logger = HybridControlLogger()
        
        # Performance monitoring
        self.performance_counter = 0
        self.total_computation_time = 0.0
        
        method_info = "EWMA + " + ("Spline" if _SCIPY_OK else "Fallback")
        self.get_logger().info(f'Hybrid Lane Control initialized: {method_info}')

    def callback_lane_state(self, msg):
        self.lane_state = msg.data

    def callback_get_max_vel(self, max_vel_msg):
        self.MAX_VEL = max_vel_msg.data

    def callback_follow_lane(self, desired_center):
        if self.avoid_active:
            return

        with TimingContext() as timer:
            center = desired_center.data
            error_raw = center - self.center_px
            current_time = time.time()

            # Get hybrid estimate
            estimate = self.hybrid_estimator.update(error_raw, current_time)

            # Extract results
            e_for_ctrl = estimate['final_filtered']
            d_for_ctrl = estimate['final_derivative']

            # PD control
            angular_z_before_sat = self.Kp * e_for_ctrl + self.Kd * d_for_ctrl

            # Apply saturation
            angular_z_output = np.clip(-angular_z_before_sat, -2.0, 2.0)

            # Velocity calculation
            error_magnitude = abs(e_for_ctrl) / self.center_px
            gain = max(1.0 - error_magnitude, 0.0) ** 2.2
            linear_x_output = min(self.MAX_VEL * gain, 0.05)

        # Performance monitoring
        computation_time = timer.get_ms()
        self.performance_counter += 1
        self.total_computation_time += computation_time
        
        if self.performance_counter % 100 == 0:
            avg_time = self.total_computation_time / 100
            confidence = estimate['confidence']
            self.get_logger().info(
                f'Avg computation: {avg_time:.4f}ms, Confidence: {confidence:.3f}, '
                f'EWMA weight: {estimate["ewma_weight"]:.2f}'
            )
            self.total_computation_time = 0.0

        # Log comprehensive data
        self.logger.log_control_data(
            lane_center=center,
            error_raw=error_raw,
            ewma_filtered=estimate['ewma_filtered'],
            spline_filtered=estimate['spline_filtered'],
            final_filtered=estimate['final_filtered'],
            ewma_derivative=estimate['ewma_derivative'],
            spline_derivative=estimate['spline_derivative'],
            final_derivative=estimate['final_derivative'],
            ewma_weight=estimate['ewma_weight'],
            spline_weight=estimate['spline_weight'],
            confidence_score=estimate['confidence'],
            angular_z_before_sat=angular_z_before_sat,
            angular_z_output=angular_z_output,
            linear_x_output=linear_x_output,
            computation_time_ms=computation_time,
            lane_state=self.lane_state,
            max_vel=self.MAX_VEL,
            spline_active=estimate['spline_active'],
            spline_samples=estimate['spline_samples']
        )

        # Publish control command
        twist = Twist()
        twist.linear.x = linear_x_output
        twist.angular.z = angular_z_output
        self.pub_cmd_vel.publish(twist)
        
        self.last_error = error_raw

    def callback_avoid_cmd(self, twist_msg):
        self.avoid_twist = twist_msg
        if self.avoid_active:
            self.pub_cmd_vel.publish(self.avoid_twist)

    def callback_avoid_active(self, bool_msg):
        self.avoid_active = bool_msg.data
        if self.avoid_active:
            self.get_logger().info('Avoidance mode activated.')
        else:
            self.get_logger().info('Avoidance mode deactivated. Returning to lane following.')

    def shut_down(self):
        self.get_logger().info('Shutting down. cmd_vel will be 0')
        self.logger._flush_buffer()  # Ensure all data is saved
        twist = Twist()
        self.pub_cmd_vel.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ControlLane()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shut_down()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
