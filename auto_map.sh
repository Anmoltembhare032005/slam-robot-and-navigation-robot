#!/usr/bin/env bash
# auto_map.sh — Autonomous Explorer for Mapping
set -e

source /opt/ros/humble/setup.bash
source /home/anmol/slam_ws/install/setup.bash

echo "============================================="
echo " 🤖 Starting Autonomous Explorer (Auto-Mapping)..."
echo "============================================="
python3 /home/anmol/slam_ws/src/slam_robot/scripts/safe_auto_mapper.py
