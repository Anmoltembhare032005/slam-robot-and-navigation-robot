#!/usr/bin/env bash
# save_map.sh — Save generated SLAM map
set -e

source /opt/ros/humble/setup.bash

echo "============================================="
echo " 💾 Saving SLAM Map..."
echo "============================================="
mkdir -p /home/anmol/slam_ws/src/slam_robot/maps
ros2 run nav2_map_server map_saver_cli -f /home/anmol/slam_ws/src/slam_robot/maps/slam_map

echo "Converting to PGM format for Nav2..."
python3 -c "from PIL import Image; Image.open('/home/anmol/slam_ws/src/slam_robot/maps/slam_map.png').save('/home/anmol/slam_ws/src/slam_robot/maps/slam_map.pgm', format='PPM')" 2>/dev/null || true

sed -i 's/slam_map.png/slam_map.pgm/g' /home/anmol/slam_ws/src/slam_robot/maps/slam_map.yaml 2>/dev/null || true

echo "✅ Map saved successfully in ~/slam_ws/src/slam_robot/maps/ !"
