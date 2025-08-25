# TurtleBot3 Autorace - Enhanced Control System

## 프로젝트 개요
TurtleBot3 자율주행 시스템의 장애물 회피 및 제어 성능 개선 프로젝트

## 최근 개선사항 (2025-08-25)

### 🎯 주요 개선 목표
기존 단순 PD 제어의 한계를 극복하고 수치적 안정성과 제어 성능을 동시에 향상

### 📋 개선사항 상세

### 8. **주차표시 탐지 및 매뉴버링 시스템**

**새로운 기능:**
주차 표지 탐지 시 자동 접근, 정렬, 정지 시퀀스

**주요 컴포넌트:**

#### 8.1 **센서 통합**
```python
# 주차 표지 탐지 신호 구독
self.detect_parking_sign_sub = self.create_subscription(
    UInt8, '/detect/traffic_sign', self.parking_sign_callback, 10)

# 주차 표지 중심 좌표 구독  
self.detect_parking_center_sub = self.create_subscription(
    Point, '/detect/parking_center', self.parking_center_callback, 10)
```

#### 8.2 **상태 기반 접근 전략**
```python
# 3단계 주차 상태 머신
parking_state = ['APPROACHING', 'STOPPING', 'STOPPED']

# 거리 추적으로 최적 정지점 결정
if self.front_distance < self.parking_closest_distance:
    self.parking_closest_distance = self.front_distance
elif self.front_distance > self.parking_closest_distance + 0.1:
    self.parking_state = 'STOPPING'  # 최근접점 통과 시 정지 시작
```

#### 8.3 **시각 정렬 제어**
```python
# 주차 표지 중심 좌표 기반 정렬
angular_error = -self.parking_center_x  # -1(left) to 1(right) 정규화
angular_z = angular_error * 0.6  # 비례 제어 게인

# 정렬 상태에 따른 속도 조절
if abs(angular_error) < self.parking_angle_threshold:
    twist.linear.x = self.parking_approach_speed  # 0.015 m/s
else:
    twist.linear.x = self.parking_approach_speed * 0.8  # 정렬 중 감속
```

#### 8.4 **시간 기반 정지 관리**
```python
# 5초간 정지 후 정상 운행 복귀
if time_elapsed >= self.parking_stop_duration:
    self.parking_maneuver_active = False
    self.state = 'NORMAL'
```

**파라미터 설정:**
- `parking_approach_speed`: 0.015 m/s (접근 속도)
- `parking_angle_threshold`: 0.1 (정렬 허용 오차)
- `parking_target_distance`: 0.6m (목표 거리)
- `parking_stop_duration`: 5.0초 (정지 시간)
- `parking_signal_timeout`: 3.0초 (신호 타임아웃)

--- 

#### 1. **미분항(D-term) 개선: Savitzky-Golay 필터 적용**

**문제점:**
- 기존: `(error - last_error)` 단순 차분으로 노이즈에 민감
- 프레임 지터와 센서 노이즈로 인한 제어 불안정

**해결책:**
```python
# 기존 코드
angular_z = self.turn_Kp * error + self.turn_Kd * (error - self.last_turn_error)

# 개선된 코드
derivative = self.calculate_filtered_derivative(error, current_time)
angular_z = self.turn_Kp * error + self.turn_Kd * derivative
```

**기술적 세부사항:**
- **Savitzky-Golay 필터**: 윈도우 크기 9, 다항식 차수 2
- **실시간 dt 반영**: 각 제어 루프에서 실제 경과 시간 계산
- **1차 LPF 적용**: `α = 0.7`로 고주파 노이즈 추가 제거
- **Fallback 메커니즘**: S-G 필터 실패 시 단순 차분으로 대체

#### 2. **PD 이산화 개선: 필터드 차분 구현**

**문제점:**
- 샘플링 지터로 인한 미분 계산 오차
- 고주파 성분 증폭으로 인한 제어기 불안정

**해결책:**
- **시간 히스토리 추적**: `deque` 구조로 최근 9개 샘플의 시간 정보 저장
- **적응적 샘플링**: 실제 샘플링 간격을 반영한 미분 계산
- **필터링된 미분**: S-G 필터로 매끄러운 도함수 추정

#### 3. **상태 전이 개선: 히스테리시스 + EWMA**

**문제점:**
- 단일 임계값으로 인한 채터링(chattering) 현상
- 급격한 상태 변화로 인한 제어 불연속

**해결책:**
```python
# 이중 임계값 (히스테리시스)
self.turn_threshold_enter = 0.05   # 다음 상태 진입 임계값
self.turn_threshold_exit = 0.08    # 현재 상태 유지 임계값

# EWMA로 오차 평활화
self.ewma_error = α * |error| + (1-α) * ewma_error_prev
```

**효과:**
- **채터링 방지**: 상태 전이 안정화
- **부드러운 전환**: EWMA (α=0.3)로 오차 평활화
- **상태 의존적 임계값**: 현재 상태에 따른 다른 기준 적용

#### 4. **각속도 포화 및 레이트 제한**

**문제점:**
- PD 제어 출력이 무제한으로 커질 수 있음
- 급격한 각속도 변화로 인한 기계적 스트레스

**해결책:**
```python
# 속도 포화 (Velocity Saturation)
max_angular_velocity = 1.0  # rad/s

# 가속도 제한 (Rate Limiting)  
max_angular_acceleration = 2.0  # rad/s²
max_change = max_acceleration * dt
vel_change = clip(desired_vel - last_vel, -max_change, max_change)
```

**기술적 구현:**
- **이중 제한**: 속도 한계 → 가속도 한계 순서로 적용
- **연속성 보장**: 이전 각속도를 기억하여 부드러운 변화
- **실시간 적용**: 실제 dt를 사용한 가속도 제한

#### 5. **직진 구간 개선: 소곡률 유지**

**문제점:**
- 완전히 0인 각속도로 직진 시 장애물 재접촉 가능성
- 경로 보정 메커니즘 부재

**해결책:**
```python
# 전방 장애물 거리 기반 소곡률 적용
if self.front_distance < 1.0:
    if self.turn_direction == 'left':
        twist.angular.z = 0.05   # 소곡률 좌회전 유지
    elif self.turn_direction == 'right':
        twist.angular.z = -0.05  # 소곡률 우회전 유지
```

**장점:**
- **안전성 향상**: 장애물과 안전 거리 유지
- **경로 안정성**: 회피 방향 일관성 유지
- **계산 효율성**: 단순한 조건문으로 구현

#### 6. **실시간 시간 추적 및 저역통과 필터**

**문제점:**
- 고정된 제어 주기 가정으로 인한 부정확성
- 제어 신호의 고주파 잡음

**해결책:**
```python
# 실시간 dt 계산
current_time = self.get_clock().now()
dt = (current_time - self.last_time).nanoseconds / 1e9

# 저역통과 필터 적용
filtered_derivative = α * raw_derivative + (1-α) * prev_filtered_derivative
```

### 🔧 기술적 아키텍처 변경사항

#### 새로운 Dependencies
```python
from collections import deque      # 시간 히스토리 관리
from scipy import signal          # Savitzky-Golay 필터
```

#### 새로운 클래스 변수
```python
# Savitzky-Golay 필터 설정
sg_window_size = 9          # 필터 윈도우 크기
sg_poly_order = 2           # 다항식 차수
error_history = deque()     # 오차 히스토리
time_history = deque()      # 시간 히스토리

# 필터링 및 평활화
filtered_derivative = 0.0   # 필터링된 미분값
lpf_alpha = 0.7            # 저역통과 필터 계수
ewma_error = 0.0           # EWMA 오차
ewma_alpha = 0.3           # EWMA 계수

# 각속도 제한
max_angular_velocity = 1.0      # 최대 각속도 (rad/s)
max_angular_acceleration = 2.0  # 최대 각가속도 (rad/s²)
last_angular_velocity = 0.0     # 이전 각속도
```

#### 새로운 메서드
1. **`calculate_filtered_derivative(error, current_time)`**
   - Savitzky-Golay 필터를 사용한 노이즈 강건 미분 계산
   - 실시간 시간 간격 반영
   - Fallback 메커니즘 포함

2. **`apply_angular_limits(desired_angular_vel, dt)`**
   - 속도 포화 및 가속도 제한 적용
   - 연속성 보장을 위한 이전 상태 기억

3. **`update_ewma_error(error)`**
   - 지수이동평균을 사용한 오차 평활화
   - 상태 전이 안정화

4. **`check_turn_completion_with_hysteresis(error, current_state)`**
   - 히스테리시스 기반 상태 전이 판정
   - 채터링 방지

### 📊 성능 개선 효과

#### 안정성 (Stability)
- **채터링 제거**: 히스테리시스로 상태 전이 안정화
- **노이즈 억제**: S-G 필터와 LPF로 제어 신호 평활화
- **포화 방지**: 각속도/가속도 제한으로 안전 보장

#### 응답성 (Responsiveness)  
- **적응적 제어**: 실시간 dt 반영으로 정확한 제어
- **최적화된 필터**: 지연 최소화한 S-G 필터 설정
- **상태 의존 임계값**: 상황에 맞는 반응성 조절

#### 재현성 (Reproducibility)
- **결정적 동작**: 노이즈 필터링으로 일관된 제어
- **상태 기억**: 제어 히스토리를 통한 연속성 보장
- **파라미터 최적화**: 경험적으로 검증된 필터 계수

### 🚀 사용법

#### 빌드 및 실행
```bash
# 워크스페이스 빌드
cd ~/turtlebot3_ws
colcon build --packages-select turtlebot3_autorace_mission

# 실행
ros2 launch turtlebot3_autorace_mission turtlebot3_autorace.launch.py
```

#### 파라미터 튜닝 가이드

**PD 게인 조정:**
- `turn_Kp`: 비례 게인 (기본값: 0.45)
- `turn_Kd`: 미분 게인 (기본값: 0.03)

**필터 설정:**
- `sg_window_size`: S-G 필터 윈도우 (기본값: 9, 홀수여야 함)
- `lpf_alpha`: 저역통과 필터 계수 (0 < α < 1, 기본값: 0.7)

**상태 전이 임계값:**
- `turn_threshold_enter`: 진입 임계값 (기본값: 0.05)
- `turn_threshold_exit`: 유지 임계값 (기본값: 0.08)

**속도 제한:**
- `max_angular_velocity`: 최대 각속도 (기본값: 1.0 rad/s)
- `max_angular_acceleration`: 최대 각가속도 (기본값: 2.0 rad/s²)

### 🔍 디버깅 및 모니터링

#### 로그 메시지
개선된 시스템은 상세한 디버그 정보를 제공합니다:
- 상태 전이 정보
- 필터링된 제어 신호 값  
- EWMA 오차 및 임계값 비교
- 각속도 제한 적용 상태

#### 시각화
기존 시각화에 추가된 정보:
- 필터링된 미분값 표시
- EWMA 오차 트렌드
- 상태 전이 히스테리시스 상태

### ⚠️ 주의사항

1. **scipy 의존성**: `scipy` 패키지가 설치되어야 함
2. **메모리 사용량**: deque 히스토리로 인한 약간의 메모리 증가
3. **초기화 시간**: 필터가 안정화되기까지 약 1-2초 소요
4. **파라미터 민감성**: 필터 계수 변경 시 신중한 테스트 필요

## 🚗 추가 개선사항

### 7. **차선 추종 제어 개선 (control_lane.py)**

**문제점:**
- 기존: 단순 차분 미분으로 노이즈 민감
- 고정 샘플링 주기 가정으로 부정확

**해결책: Cubic Spline 기반 미분 추정**
```python
# 스플라인 기반 미분기 클래스
class SplineDifferentiator:
    def __init__(self, maxlen=30, bc_type='natural'):
        self.t_buf = deque(maxlen=maxlen)
        self.e_buf = deque(maxlen=maxlen)
        
    def add(self, t_sec, e_val):
        # 최근 30개 (시간, 오차) 샘플 버퍼링
        
    def eval(self, t_now):
        # CubicSpline 적합 후 S(t_now), S'(t_now) 반환
```

**기술적 세부사항:**
- **CubicSpline 적합**: `scipy.interpolate.CubicSpline`으로 매끄러운 보간
- **Natural 경계조건**: 끝점에서 2차 도함수 = 0
- **시간 정렬 & 중복 제거**: `np.argsort()` + unique 필터링
- **안전한 외삽 방지**: 경계 클램핑으로 extrapolation 제한
- **Fallback 메커니즘**: SciPy 없으면 기존 차분으로 자동 전환

**게인 보존 설계:**
```python
# 기존 Kd 튜닝 보존을 위한 스케일 조정
d_for_ctrl = de_dt * self.dt_nom  # 연속시간 도함수 → 이산시간 차분 스케일
```



### 📊 통합 시스템 아키텍처

#### 우선순위 기반 상태 관리
```python
def process_loop(self):
    # 1순위: 주차 매뉴버
    if self.should_handle_parking_maneuver():
        self.process_parking_maneuver_state()
    # 2순위: 신호등 제어    
    elif self.should_handle_traffic_light():
        self.process_traffic_light_state()
    # 3순위: 일반/장애물 회피
    else:
        # NORMAL, AVOID_TURN, AVOID_STRAIGHT, RETURN_TURN
```

#### 시각화 개선
```python
# 주차 상태 정보 오버레이
parking_text = f'Parking: {self.parking_state}'
parking_center_text = f'P-Center: ({self.parking_center_x:.2f}, {self.parking_center_y:.2f})'
parking_distance_text = f'P-Closest: {self.parking_closest_distance:.2f}m'
```

### 🔧 기술적 의존성 업데이트

#### control_lane.py 추가 요구사항
```python
from collections import deque
import numpy as np
from scipy.interpolate import CubicSpline  # 선택적 의존성
```

#### avoid_construction.py 확장 토픽
```python
# 새로운 구독 토픽
'/detect/traffic_sign'    # UInt8: 주차 표지 탐지 신호
'/detect/parking_center'  # Point: 주차 표지 중심 좌표 (정규화됨)
```

### 📈 향후 개선 계획

1. **적응적 파라미터**: 환경에 따른 자동 게인 조정
2. **머신러닝 통합**: 학습 기반 제어 파라미터 최적화
3. **예측 제어**: 모델 예측 제어(MPC) 도입 검토
4. **멀티센서 융합**: IMU, 비전 정보 통합 제어
5. **동적 주차**: 다양한 주차 시나리오 대응 알고리즘
6. **실시간 캘리브레이션**: 온라인 파라미터 튜닝

---

## 파일 구조
```
src/turtlebot3_autorace/turtlebot3_autorace_mission/turtlebot3_autorace_mission/
├── avoid_construction.py  # 🔄 대폭 개선된 메인 제어 로직 + 주차 매뉴버링
├── control_lane.py        # 🔄 Spline 기반 차선 추종 제어
└── ...
```

## 기여자
- **제어 시스템 개선**: Claude Code Assistant (2025-08-25)
- **수치 해석 자문**: 사용자 제공 기술 분석

## 라이선스
Apache License 2.0
