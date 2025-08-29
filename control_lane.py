# #!/usr/bin/env python3
# #
# # (원저작권 표기 동일)
# #
# # 변경: D항(미분) 계산을 차분 → 스플라인 보간 기반으로 개선
# #  - CubicSpline으로 최근 오류 시계열 e(t)를 적합하고, S'(t_now)로 d항을 추정
# #  - SciPy가 없으면 기존 차분으로 자동 폴백
# #  - dt_nom(기대 주기)로 스케일 맞춰 기존 Kd 튜닝을 크게 바꾸지 않도록 함

# from geometry_msgs.msg import Twist
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Bool
# from std_msgs.msg import Float64

# # --- Added: 필요한 유틸 ---
# from collections import deque
# import numpy as np
# try:
#     from scipy.interpolate import CubicSpline
#     _SCIPY_OK = True
# except Exception:
#     CubicSpline = None
#     _SCIPY_OK = False


# # --- Added: 스플라인 기반 미분기 추정기 ---
# class SplineDifferentiator:
#     """
#     최근 N개 (t_i, e_i)를 버퍼링하고 CubicSpline으로 적합한 뒤,
#     현재 시각 t_now에서 값 S(t_now), 미분 S'(t_now)를 반환.
#     """
#     def __init__(self, maxlen=30, bc_type='natural'):
#         self.t_buf = deque(maxlen=maxlen)
#         self.e_buf = deque(maxlen=maxlen)
#         self.bc_type = bc_type
#         self.spline = None

#     def add(self, t_sec: float, e_val: float):
#         self.t_buf.append(float(t_sec))
#         self.e_buf.append(float(e_val))
#         # 샘플이 충분하고, 시간 범위가 의미있을 때만 적합
#         if len(self.t_buf) >= 5:
#             t = np.array(self.t_buf, dtype=float)
#             e = np.array(self.e_buf, dtype=float)

#             # 시간 증가 정렬 + 중복 타임스탬프 제거
#             idx = np.argsort(t)
#             t, e = t[idx], e[idx]
#             uniq = np.r_[True, np.diff(t) > 1e-6]
#             t, e = t[uniq], e[uniq]

#             if len(t) >= 5 and (t[-1] - t[0]) > 1e-3:
#                 self.spline = CubicSpline(t, e, bc_type=self.bc_type)

#     def eval(self, t_now: float):
#         """
#         스플라인이 준비되면 (e_hat, de_dt) 반환, 아니면 (None, None)
#         외삽은 위험하므로 경계 클램프
#         """
#         if self.spline is None:
#             return None, None
#         t0, t1 = self.spline.x[0], self.spline.x[-1]
#         tt = min(max(float(t_now), t0), t1)
#         e_hat = float(self.spline(tt))
#         de_dt = float(self.spline(tt, 1))
#         return e_hat, de_dt


# class ControlLane(Node):

#     def __init__(self):
#         super().__init__('control_lane')

#         self.sub_lane = self.create_subscription(
#             Float64,
#             '/control/lane',
#             self.callback_follow_lane,
#             1
#         )
#         self.sub_max_vel = self.create_subscription(
#             Float64,
#             '/control/max_vel',
#             self.callback_get_max_vel,
#             1
#         )
#         self.sub_avoid_cmd = self.create_subscription(
#             Twist,
#             '/avoid_control',
#             self.callback_avoid_cmd,
#             1
#         )
#         self.sub_avoid_active = self.create_subscription(
#             Bool,
#             '/avoid_active',
#             self.callback_avoid_active,
#             1
#         )

#         self.pub_cmd_vel = self.create_publisher(Twist, '/control/cmd_vel', 1)

#         # PD control related variables
#         self.last_error = 0.0
#         self.MAX_VEL = 0.1

#         # Avoidance mode related variables
#         self.avoid_active = False
#         self.avoid_twist = Twist()

#         # --- Added: 스플라인 설정/버퍼 ---
#         self.center_px = 500.0        # 기존 하드코딩 유지
#         self.dt_nom = 0.05            # 기대 샘플 주기(초) — d항 스케일 보정용
#         self.spline_diff = None
#         if _SCIPY_OK:
#             self.spline_diff = SplineDifferentiator(maxlen=30, bc_type='natural')
#             self.get_logger().info('[control_lane] Using spline-based derivative (SciPy OK).')
#         else:
#             self.get_logger().warn('[control_lane] SciPy not found. Falling back to finite-difference.')

#         # 게인 (기존 값 유지)
#         self.Kp = 0.0025
#         self.Kd = 0.007

#     def callback_get_max_vel(self, max_vel_msg):
#         self.MAX_VEL = max_vel_msg.data

#     def _now_sec(self):
#         return self.get_clock().now().nanoseconds * 1e-9

#     def callback_follow_lane(self, desired_center):
#         """
#         Receive lane center data to generate lane following control commands.
#         If avoidance mode is enabled, lane following control is ignored.
#         """
#         if self.avoid_active:
#             return

#         center = desired_center.data
#         error_raw = float(center - self.center_px)

#         # --- Added: 스플라인 버퍼 갱신 & 평가 ---
#         use_spline = _SCIPY_OK and (self.spline_diff is not None)
#         e_for_ctrl = error_raw
#         d_for_ctrl = (error_raw - self.last_error)  # 기본: 차분(Δe)

#         if use_spline:
#             t_now = self._now_sec()
#             self.spline_diff.add(t_now, error_raw)
#             e_hat, de_dt = self.spline_diff.eval(t_now)
#             if e_hat is not None:
#                 # e_hat = 스무딩된 오류, de_dt = 연속 시간 도함수
#                 # Kd 튜닝 보존을 위해 Δe 스케일로 환산: de_dt * dt_nom ≈ Δe
#                 e_for_ctrl = e_hat
#                 d_for_ctrl = de_dt * self.dt_nom

#         # PD 제어
#         angular_z_raw = self.Kp * e_for_ctrl + self.Kd * d_for_ctrl

#         twist = Twist()
#         # Linear velocity: adjust speed based on error (maximum 0.05 limit)
#         gain = max(1.0 - abs(e_for_ctrl) / self.center_px, 0.0) ** 2.2
#         twist.linear.x = min(self.MAX_VEL * gain, 0.05)

#         # Saturation & sign convention 유지
#         if angular_z_raw < 0:
#             twist.angular.z = -max(angular_z_raw, -2.0)
#         else:
#             twist.angular.z = -min(angular_z_raw, 2.0)

#         self.pub_cmd_vel.publish(twist)
#         self.last_error = error_raw  # 원시 오류 저장(폴백/모니터링용)

#     def callback_avoid_cmd(self, twist_msg):
#         self.avoid_twist = twist_msg
#         if self.avoid_active:
#             self.pub_cmd_vel.publish(self.avoid_twist)

#     def callback_avoid_active(self, bool_msg):
#         self.avoid_active = bool_msg.data
#         if self.avoid_active:
#             self.get_logger().info('Avoidance mode activated.')
#         else:
#             self.get_logger().info('Avoidance mode deactivated. Returning to lane following.')

#     def shut_down(self):
#         self.get_logger().info('Shutting down. cmd_vel will be 0')
#         twist = Twist()
#         self.pub_cmd_vel.publish(twist)


# def main(args=None):
#     rclpy.init(args=args)
#     node = ControlLane()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.shut_down()
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()

# """----------차분 미분 버전------------"""
# #!/usr/bin/env python3
# #
# # Copyright 2018 ROBOTIS CO., LTD.
# #
# # Licensed under the Apache License, Version 2.0 (the "License");
# # you may not use this file except in compliance with the License.
# # You may obtain a copy of the License at
# #
# #     http://www.apache.org/licenses/LICENSE-2.0
# #
# # Unless required by applicable law or agreed to in writing, software
# # distributed under the License is distributed on an "AS IS" BASIS,
# # WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# # See the License for the specific language governing permissions and
# # limitations under the License.
# #
# # Author: Leon Jung, Gilbert, Ashe Kim, Hyungyu Kim, ChanHyeong Lee
# # Modified: Added comprehensive logging for simple difference derivative comparison

# from geometry_msgs.msg import Twist
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Bool
# from std_msgs.msg import Float64
# from std_msgs.msg import UInt8

# # Logging modules
# import csv
# import os
# import time
# import threading
# from datetime import datetime
# import numpy as np


# class LaneControlLogger:
#     def __init__(self, log_dir="lane_control_logs"):
#         self.log_dir = log_dir
#         os.makedirs(log_dir, exist_ok=True)
        
#         # 실행 시작 시간으로 파일명 구분
#         timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
#         self.csv_file = os.path.join(log_dir, f"simple_diff_control_{timestamp}.csv")
        
#         # CSV 헤더
#         self.csv_headers = [
#             'timestamp',           # 절대 시간 (초)
#             'relative_time',       # 시작점 대비 상대 시간 (초)
#             'method',              # 'simple_diff'
#             'lane_center',         # lane detection에서 받은 center 값
#             'error_raw',           # center - 500 (픽셀 단위)
#             'derivative_raw',      # 원시 미분값 (차분)
#             'angular_z_before_sat',# 포화 전 각속도 명령
#             'angular_z_output',    # 최종 출력 각속도
#             'linear_x_output',     # 선속도 출력
#             'computation_time_ms', # 계산 시간 (ms)
#             'sample_interval_ms',  # 실제 샘플링 간격
#             'control_effort',      # |angular_z - prev_angular_z|
#             'lane_state',          # detect_lane에서 받은 상태
#             'max_vel_setting'      # 현재 MAX_VEL 설정값
#         ]
        
#         # 파일 초기화
#         with open(self.csv_file, 'w', newline='') as f:
#             writer = csv.writer(f)
#             writer.writerow(self.csv_headers)
        
#         # 스레드 안전 및 상태 변수
#         self.lock = threading.Lock()
#         self.start_time = time.time()
#         self.prev_angular_z = 0.0
#         self.prev_timestamp = None
        
#         print(f"[LOGGER] Simple diff logging to: {self.csv_file}")
        
#     def log_control_data(self, lane_center, error_raw, derivative_raw, 
#                         angular_z_before_sat, angular_z_output, linear_x_output, 
#                         computation_time_ms, lane_state=None, max_vel=None):
        
#         with self.lock:
#             current_time = time.time()
#             relative_time = current_time - self.start_time
            
#             # 제어 노력 계산
#             control_effort = abs(angular_z_output - self.prev_angular_z)
#             self.prev_angular_z = angular_z_output
            
#             # 샘플링 간격 계산
#             sample_interval_ms = 0.0
#             if self.prev_timestamp is not None:
#                 sample_interval_ms = (current_time - self.prev_timestamp) * 1000
#             self.prev_timestamp = current_time
            
#             # CSV 행 데이터
#             row_data = [
#                 current_time,
#                 relative_time,
#                 'simple_diff',
#                 lane_center,
#                 error_raw,
#                 derivative_raw,
#                 angular_z_before_sat,
#                 angular_z_output,
#                 linear_x_output,
#                 computation_time_ms,
#                 sample_interval_ms,
#                 control_effort,
#                 lane_state if lane_state is not None else '',
#                 max_vel if max_vel is not None else ''
#             ]
            
#             # CSV 파일에 기록
#             with open(self.csv_file, 'a', newline='') as f:
#                 writer = csv.writer(f)
#                 writer.writerow(row_data)


# # 계산 시간 측정용 컨텍스트 매니저
# class TimingContext:
#     def __init__(self):
#         self.start = None
#         self.end = None
    
#     def __enter__(self):
#         self.start = time.perf_counter()
#         return self
    
#     def __exit__(self, *args):
#         self.end = time.perf_counter()
    
#     def get_ms(self):
#         return (self.end - self.start) * 1000 if self.start and self.end else 0.0


# class ControlLane(Node):

#     def __init__(self):
#         super().__init__('control_lane')

#         self.sub_lane = self.create_subscription(
#             Float64,
#             '/control/lane',
#             self.callback_follow_lane,
#             1
#         )
#         self.sub_max_vel = self.create_subscription(
#             Float64,
#             '/control/max_vel',
#             self.callback_get_max_vel,
#             1
#         )
#         self.sub_avoid_cmd = self.create_subscription(
#             Twist,
#             '/avoid_control',
#             self.callback_avoid_cmd,
#             1
#         )
#         self.sub_avoid_active = self.create_subscription(
#             Bool,
#             '/avoid_active',
#             self.callback_avoid_active,
#             1
#         )
        
#         # lane_state 구독 추가 (로깅용)
#         self.sub_lane_state = self.create_subscription(
#             UInt8, '/detect/lane_state', self.callback_lane_state, 1)

#         self.pub_cmd_vel = self.create_publisher(
#             Twist,
#             '/control/cmd_vel',
#             1
#         )

#         # PD control related variables
#         self.last_error = 0.0
#         self.MAX_VEL = 0.1

#         # Avoidance mode related variables
#         self.avoid_active = False
#         self.avoid_twist = Twist()
        
#         # 로깅을 위한 추가 변수
#         self.lane_state = None
        
#         # 로거 초기화
#         self.logger = LaneControlLogger()
        
#         self.get_logger().info('Simple Difference Control Lane node started with logging')

#     def callback_lane_state(self, msg):
#         self.lane_state = msg.data

#     def callback_get_max_vel(self, max_vel_msg):
#         self.MAX_VEL = max_vel_msg.data

#     def callback_follow_lane(self, desired_center):
#         """
#         Receive lane center data to generate lane following control commands.
#         If avoidance mode is enabled, lane following control is ignored.
#         """
#         if self.avoid_active:
#             return

#         # 타이밍 시작
#         with TimingContext() as timer:
#             center = desired_center.data
#             error_raw = center - 500
            
#             # 차분 미분 계산
#             derivative_raw = error_raw - self.last_error
            
#             # PD 제어 계산
#             Kp = 0.0025
#             Kd = 0.007
#             angular_z_before_sat = Kp * error_raw + Kd * derivative_raw
            
#             # 포화 적용
#             if angular_z_before_sat < 0:
#                 angular_z_output = -max(angular_z_before_sat, -2.0)
#             else:
#                 angular_z_output = -min(angular_z_before_sat, 2.0)
            
#             # 선속도 계산
#             linear_x_output = min(self.MAX_VEL * (max(1 - abs(error_raw) / 500, 0) ** 2.2), 0.05)
        
#         # 로깅
#         self.logger.log_control_data(
#             lane_center=center,
#             error_raw=error_raw,
#             derivative_raw=derivative_raw,
#             angular_z_before_sat=angular_z_before_sat,
#             angular_z_output=angular_z_output,
#             linear_x_output=linear_x_output,
#             computation_time_ms=timer.get_ms(),
#             lane_state=self.lane_state,
#             max_vel=self.MAX_VEL
#         )
        
#         # 제어 명령 발행
#         twist = Twist()
#         twist.linear.x = linear_x_output
#         twist.angular.z = angular_z_output
#         self.pub_cmd_vel.publish(twist)
        
#         self.last_error = error_raw

#     def callback_avoid_cmd(self, twist_msg):
#         self.avoid_twist = twist_msg
#         if self.avoid_active:
#             self.pub_cmd_vel.publish(self.avoid_twist)

#     def callback_avoid_active(self, bool_msg):
#         self.avoid_active = bool_msg.data
#         if self.avoid_active:
#             self.get_logger().info('Avoidance mode activated.')
#         else:
#             self.get_logger().info('Avoidance mode deactivated. Returning to lane following.')

#     def shut_down(self):
#         self.get_logger().info('Shutting down. cmd_vel will be 0')
#         twist = Twist()
#         self.pub_cmd_vel.publish(twist)


# def main(args=None):
#     rclpy.init(args=args)
#     node = ControlLane()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.shut_down()
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()

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


class EWMAEstimator:
    """Exponentially Weighted Moving Average estimator"""
    def __init__(self, alpha=0.3, dt_nom=0.033):
        self.alpha = alpha  # Smoothing factor (0 < alpha < 1)
        self.dt_nom = dt_nom
        self.smoothed_error = None
        self.prev_smoothed_error = None
        # EWMAEstimator.update
        now = time.perf_counter()
        if self._last is None: self._last = now
        dt = max(now - self._last, 1e-6); self._last = now
        
        
    def update(self, error):
        if self.smoothed_error is None:
            self.smoothed_error = error
            self.prev_smoothed_error = error
            self.derivative = 0.0
            return self.smoothed_error, self.derivative
        
        # Update smoothed error
        self.prev_smoothed_error = self.smoothed_error
        self.smoothed_error = self.alpha * error + (1 - self.alpha) * self.smoothed_error
        
        # Compute derivative
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