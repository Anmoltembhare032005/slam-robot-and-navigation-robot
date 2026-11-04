#!/usr/bin/env bash
# launch_sim.sh — Launch Ignition Gazebo Fortress Simulation
set -e

source /opt/ros/humble/setup.bash
source /home/anmol/slam_ws/install/setup.bash

echo "============================================="
echo " 🚀 Launching SLAM Robot Simulation in Gazebo..."
echo "============================================="
ros2 launch slam_robot sim.launch.py
