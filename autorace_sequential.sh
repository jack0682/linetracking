#!/bin/bash

# TurtleBot3 AutoRace Sequential Launch Script
# 각 명령어를 순서대로 실행하며 3초씩 딜레이를 줍니다

echo "TurtleBot3 AutoRace 순차 실행 스크립트 시작..."
echo "TURTLEBOT3_MODEL 환경변수가 설정되어 있는지 확인하세요 (예: export TURTLEBOT3_MODEL=burger)"
cd ~/turtlebot3_ws
rm -rf build
rm -rf install
rm -rf log
sleep 3

colcon build
source install/setup.bash

echo "1. Gazebo 시작 중..."
ros2 launch turtlebot3_gazebo turtlebot3_autorace_2020.launch.py &
GAZEBO_PID=$!
sleep 5

echo "2. Intrinsic Camera Calibration 시작 중..."
ros2 launch turtlebot3_autorace_camera intrinsic_camera_calibration.launch.py &
INTRINSIC_PID=$!
sleep 5

echo "3. Extrinsic Camera Calibration 시작 중..."
ros2 launch turtlebot3_autorace_camera extrinsic_camera_calibration.launch.py calibration_mode:=True &
EXTRINSIC_PID=$!
sleep 5

echo "4. Lane Detection 시작 중..."
ros2 launch turtlebot3_autorace_detect detect_lane.launch.py &
LANE_DETECT_PID=$!
sleep 3

echo "5. Lane Control 시작 중..."
ros2 launch turtlebot3_autorace_mission control_lane.launch.py &
LANE_CONTROL_PID=$!
sleep 3

echo "6. Mission Construction 시작 중..."
ros2 launch turtlebot3_autorace_mission mission_construction.launch.py &
MISSION_PID=$!

echo "7. Sign Detection 시작 중..."
ros2 launch turtlebot3_autorace_detect detect_sign.launch.py &
SIGN_DETECT_PID=$!
sleep 3

echo "8. Traffic Light Detection 시작 중..."
ros2 launch turtlebot3_autorace_detect detect_traffic_light.launch.py &
TRAFFIC_LIGHT_PID=$!
sleep 3


echo "모든 컴포넌트가 시작되었습니다!"
echo "종료하려면 Ctrl+C를 누르세요."

# 시그널 핸들러 - Ctrl+C 시 모든 프로세스 종료
trap 'echo "종료 중..."; kill $GAZEBO_PID $INTRINSIC_PID $EXTRINSIC_PID $LANE_DETECT_PID $LANE_CONTROL_PID $SIGN_DETECT_PID $TRAFFIC_LIGHT_PID $MISSION_PID 2>/dev/null; exit' SIGINT

# 백그라운드 프로세스들이 종료될 때까지 대기
wait