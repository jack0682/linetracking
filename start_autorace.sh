#!/bin/bash
# TurtleBot3 AutoRace Full Launch Script
# This script launches all TurtleBot3 AutoRace components in sequence

echo "Starting TurtleBot3 AutoRace Full System..."
echo "Make sure to set TURTLEBOT3_MODEL environment variable (e.g., export TURTLEBOT3_MODEL=burger)"

# Source the workspace
source /home/rokey1/turtlebot3_ws/install/setup.bash

# Launch all autorace components
ros2 launch turtlebot3_autorace_mission full_autorace.launch.py