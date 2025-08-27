#!/usr/bin/env python3
#
# (원저작권 표기 동일)
#
# 변경: D항(미분) 계산을 차분 → 스플라인 보간 기반으로 개선
#  - CubicSpline으로 최근 오류 시계열 e(t)를 적합하고, S'(t_now)로 d항을 추정
#  - SciPy가 없으면 기존 차분으로 자동 폴백
#  - dt_nom(기대 주기)로 스케일 맞춰 기존 Kd 튜닝을 크게 바꾸지 않도록 함

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Float64

# --- Added: 필요한 유틸 ---
from collections import deque
import numpy as np
try:
    from scipy.interpolate import CubicSpline
    _SCIPY_OK = True
except Exception:
    CubicSpline = None
    _SCIPY_OK = False


# --- Added: 스플라인 기반 미분기 추정기 ---
class SplineDifferentiator:
    """
    최근 N개 (t_i, e_i)를 버퍼링하고 CubicSpline으로 적합한 뒤,
    현재 시각 t_now에서 값 S(t_now), 미분 S'(t_now)를 반환.
    """
    def __init__(self, maxlen=30, bc_type='natural'):
        self.t_buf = deque(maxlen=maxlen)
        self.e_buf = deque(maxlen=maxlen)
        self.bc_type = bc_type
        self.spline = None

    def add(self, t_sec: float, e_val: float):
        self.t_buf.append(float(t_sec))
        self.e_buf.append(float(e_val))
        # 샘플이 충분하고, 시간 범위가 의미있을 때만 적합
        if len(self.t_buf) >= 5:
            t = np.array(self.t_buf, dtype=float)
            e = np.array(self.e_buf, dtype=float)

            # 시간 증가 정렬 + 중복 타임스탬프 제거
            idx = np.argsort(t)
            t, e = t[idx], e[idx]
            uniq = np.r_[True, np.diff(t) > 1e-6]
            t, e = t[uniq], e[uniq]

            if len(t) >= 5 and (t[-1] - t[0]) > 1e-3:
                self.spline = CubicSpline(t, e, bc_type=self.bc_type)

    def eval(self, t_now: float):
        """
        스플라인이 준비되면 (e_hat, de_dt) 반환, 아니면 (None, None)
        외삽은 위험하므로 경계 클램프
        """
        if self.spline is None:
            return None, None
        t0, t1 = self.spline.x[0], self.spline.x[-1]
        tt = min(max(float(t_now), t0), t1)
        e_hat = float(self.spline(tt))
        de_dt = float(self.spline(tt, 1))
        return e_hat, de_dt


class ControlLane(Node):

    def __init__(self):
        super().__init__('control_lane')

        self.sub_lane = self.create_subscription(
            Float64,
            '/control/lane',
            self.callback_follow_lane,
            1
        )
        self.sub_max_vel = self.create_subscription(
            Float64,
            '/control/max_vel',
            self.callback_get_max_vel,
            1
        )
        self.sub_avoid_cmd = self.create_subscription(
            Twist,
            '/avoid_control',
            self.callback_avoid_cmd,
            1
        )
        self.sub_avoid_active = self.create_subscription(
            Bool,
            '/avoid_active',
            self.callback_avoid_active,
            1
        )

        self.pub_cmd_vel = self.create_publisher(Twist, '/control/cmd_vel', 1)

        # PD control related variables
        self.last_error = 0.0
        self.MAX_VEL = 0.1

        # Avoidance mode related variables
        self.avoid_active = False
        self.avoid_twist = Twist()

        # --- Added: 스플라인 설정/버퍼 ---
        self.center_px = 500.0        # 기존 하드코딩 유지
        self.dt_nom = 0.05            # 기대 샘플 주기(초) — d항 스케일 보정용
        self.spline_diff = None
        if _SCIPY_OK:
            self.spline_diff = SplineDifferentiator(maxlen=30, bc_type='natural')
            self.get_logger().info('[control_lane] Using spline-based derivative (SciPy OK).')
        else:
            self.get_logger().warn('[control_lane] SciPy not found. Falling back to finite-difference.')

        # 게인 (기존 값 유지)
        self.Kp = 0.0025
        self.Kd = 0.007

    def callback_get_max_vel(self, max_vel_msg):
        self.MAX_VEL = max_vel_msg.data

    def _now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def callback_follow_lane(self, desired_center):
        """
        Receive lane center data to generate lane following control commands.
        If avoidance mode is enabled, lane following control is ignored.
        """
        if self.avoid_active:
            return

        center = desired_center.data
        error_raw = float(center - self.center_px)

        # --- Added: 스플라인 버퍼 갱신 & 평가 ---
        use_spline = _SCIPY_OK and (self.spline_diff is not None)
        e_for_ctrl = error_raw
        d_for_ctrl = (error_raw - self.last_error)  # 기본: 차분(Δe)

        if use_spline:
            t_now = self._now_sec()
            self.spline_diff.add(t_now, error_raw)
            e_hat, de_dt = self.spline_diff.eval(t_now)
            if e_hat is not None:
                # e_hat = 스무딩된 오류, de_dt = 연속 시간 도함수
                # Kd 튜닝 보존을 위해 Δe 스케일로 환산: de_dt * dt_nom ≈ Δe
                e_for_ctrl = e_hat
                d_for_ctrl = de_dt * self.dt_nom

        # PD 제어
        angular_z_raw = self.Kp * e_for_ctrl + self.Kd * d_for_ctrl

        twist = Twist()
        # Linear velocity: adjust speed based on error (maximum 0.05 limit)
        gain = max(1.0 - abs(e_for_ctrl) / self.center_px, 0.0) ** 2.2
        twist.linear.x = min(self.MAX_VEL * gain, 0.05)

        # Saturation & sign convention 유지
        if angular_z_raw < 0:
            twist.angular.z = -max(angular_z_raw, -2.0)
        else:
            twist.angular.z = -min(angular_z_raw, 2.0)

        self.pub_cmd_vel.publish(twist)
        self.last_error = error_raw  # 원시 오류 저장(폴백/모니터링용)

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