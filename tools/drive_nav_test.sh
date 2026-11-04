#!/bin/bash
# Drive test: send nav goal and record odom position every 5 s
source /opt/ros/humble/setup.bash
source ~/slam_ws/install/setup.bash

timeout 60 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 3.0, y: -1.0}, orientation: {x: 0.0, y: 0.0, z: -0.707, w: 0.707}}}}" \
  > /tmp/opencode/nav_result.txt 2>&1 &

for i in 1 2 3 4 5 6 7 8; do
  sleep 5
  echo "t=$((i*5))"
  timeout 5 ros2 topic echo /odom --field pose.pose.position --once 2>/dev/null | head -3
done
wait
echo RESULT:
tail -4 /tmp/opencode/nav_result.txt
