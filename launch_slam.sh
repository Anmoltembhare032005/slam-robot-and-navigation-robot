#!/usr/bin/env bash
# launch_slam.sh — Launch SLAM Toolbox & RViz for Mapping
set -e

source /opt/ros/humble/setup.bash
source /home/anmol/slam_ws/install/setup.bash

echo "============================================="
echo " 🗺️  Launching SLAM Toolbox Mapping + RViz..."
echo "============================================="
ros2 launch slam_robot slam.launch.py
