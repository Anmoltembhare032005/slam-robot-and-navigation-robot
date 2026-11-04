#!/usr/bin/env bash
# auto_nav.sh — Autonomous Nav2 Mission Controller
source /opt/ros/humble/setup.bash
source /home/anmol/slam_ws/install/setup.bash

MODE="${1:-patrol}"

echo "============================================="
echo " 🎯 Running Autonomous Navigation ($MODE mode)..."
echo "============================================="
python3 /home/anmol/slam_ws/src/slam_robot/scripts/navigate_auto.py "$@"
