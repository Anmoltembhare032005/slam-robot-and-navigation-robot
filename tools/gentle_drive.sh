#!/bin/bash
# Gentle coverage drive (no slip): straight legs with slow turns, LOTS of laps.
source /opt/ros/humble/setup.bash
source ~/slam_ws/install/setup.bash
pub=$(which ros2)
run() { timeout $2 $pub topic pub /cmd_vel geometry_msgs/msg/Twist "$1" -r 20 >/dev/null 2>&1; }
straight() { run "{linear: {x: 0.15}, angular: {z: 0.0}}" "$1"; }
turn() { run "{linear: {x: 0.0}, angular: {z: $1}}" "$2"; }

# leg A: north to east side sweep
straight 5.0; turn 1.57 1.6
straight 5.0; turn -1.57 1.6
straight 5.0; turn -1.57 1.6
straight 10.0; turn -1.57 1.6
straight 5.0; turn 1.57 1.6
straight 5.0; turn 1.57 1.6
straight 5.0; turn 1.57 1.6
straight 10.0; turn 1.57 1.6
straight 5.0; turn -1.57 1.6
straight 5.0; turn -1.57 1.6
straight 5.0; turn -1.57 1.6
straight 10.0; turn -1.57 1.6
straight 5.0; turn 1.57 1.6
straight 5.0; turn -1.57 1.6
straight 5.0; turn -1.57 1.6
straight 10.0; turn -1.57 1.6
run "{linear: {x: 0.0}, angular: {z: 0.0}}" 1
echo GENTLE_DRIVE_DONE
