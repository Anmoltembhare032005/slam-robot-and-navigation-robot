#!/bin/bash
# Systematic coverage drive for SLAM mapping.
source /opt/ros/humble/setup.bash
source ~/slam_ws/install/setup.bash

pub=$(which ros2)
run() { timeout $2 $pub topic pub /cmd_vel geometry_msgs/msg/Twist "$1" -r 20 >/dev/null 2>&1; }
stop() { timeout 1 $pub topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" -r 20 >/dev/null 2>&1; }

# ; full square RT loop twice + diagonal sweep
for lap in 1 2 3; do
  run "{linear: {x: -0.25}, angular: {z: 0.45}}" 7.0   # arc right-back (south half)
  stop
  run "{linear: {x: 0.25}, angular: {z: 0.5}}" 7.0    # arc right (east side)
  stop
  run "{linear: {x: 0.1}, angular: {z: -0.5}}" 7.0    # arc left (north side)
  stop
  run "{linear: {x: -0.2}, angular: {z: -0.45}}" 7.0  # arc left-back (west side)
  stop
done
run "{linear: {x: 0.28}, angular: {z: 0.25}}" 6.0
stop
run "{linear: {x: 0.3}, angular: {z: -0.3}}" 8.0
stop
run "{linear: {x: 0.15}, angular: {z: 0.6}}" 6.0
stop
run "{linear: {x: 0.28}, angular: {z: -0.25}}" 8.0
stop
stop
echo DRIVE_DONE
