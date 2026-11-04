#!/usr/bin/env bash
# launch_nav.sh — Launch Nav2 (AMCL + Costmaps + Planner) & RViz
set -e

source /opt/ros/humble/setup.bash
source /home/anmol/slam_ws/install/setup.bash

echo "============================================="
echo " 🧭 Launching Nav2 Autonomous Navigation + RViz..."
echo "============================================="
ros2 launch slam_robot nav.launch.py
