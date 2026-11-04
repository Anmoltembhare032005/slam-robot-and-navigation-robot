#!/usr/bin/env bash
# teleop.sh — Keyboard Drive Controller (Constant Speed, Zero-Jerk)
set -e

source /opt/ros/humble/setup.bash
source /home/anmol/slam_ws/install/setup.bash

python3 /home/anmol/slam_ws/src/slam_robot/scripts/keyboard_control.py
