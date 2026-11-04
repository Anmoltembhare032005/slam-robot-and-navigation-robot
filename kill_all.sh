#!/bin/bash
# Thorough cleanup of ALL sim/SLAM/Nav2/manual leftovers.
PATTERNS="ign gazebo|gz-sim|parameter_bridge|slam_toolbox|async_slam|nav2|component_container|amcl|rviz|teleop|map_saver|sim.launch.py|slam.launch.py|nav.launch.py|bringup|robot_state_publisher|gz_bridge|ros2 launch|ros2cli.daemon|lifecycle"
ps -eo pid,cmd | grep -E "$PATTERNS" | grep -v "kill_all.sh" | grep -v grep | awk '{print $1}' | sort -u | while read -r p; do kill -9 "$p" 2>/dev/null; done
sleep 2
ps -eo pid,cmd | grep -E "$PATTERNS" | grep -v "kill_all.sh" | grep -v grep | awk '{print $1}' | sort -u | while read -r p; do kill -9 "$p" 2>/dev/null; done
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_port* /tmp/fastrtps_* 2>/dev/null
sleep 1
echo "REMAINING:"
ps -eo cmd | grep -E "$PATTERNS" | grep -v "kill_all.sh" | grep -v grep | head -5
echo CLEAN_COMPLETE
