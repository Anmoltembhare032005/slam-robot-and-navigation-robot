#!/usr/bin/env bash
# sync.sh — Gazebo-Guided RViz Auto-Aligner
# Snaps RViz robot pose & LiDAR scan directly to Gazebo Ground Truth
set -e

export FASTRTPS_DEFAULT_PROFILES_FILE=/home/anmol/slam_ws/src/slam_robot/config/fastrtps_udp.xml
source /opt/ros/humble/setup.bash
source /home/anmol/slam_ws/install/setup.bash

python3 /home/anmol/slam_ws/tools/auto_align.py
