#!/bin/bash
# Odom drift test: drive straight 6 s at 0.3 m/s, compare /odom with gazebo ground truth.
source /opt/ros/humble/setup.bash
source ~/slam_ws/install/setup.bash

# reset robot near origin for repeatable test
timeout 15 ign service -s /world/slam_room/set_entity_pose --reqtype ignition.msgs.SetPose --reptype ignition.msgs.SetPoseReply --timeout 10000 --req 'name: "slam_robot", pose: {position: {x: 0.0, y: 0.0, z: 0.1}}' >/dev/null 2>&1
sleep 2

timeout 4 ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" -r 20 >/dev/null 2>&1
echo "=== odom BEFORE ==="
timeout 6 ros2 topic echo /odom --field pose.pose.position --once 2>/dev/null | head -3
echo "=== gz BEFORE ==="
timeout 6 ign topic -t /world/slam_room/dynamic_pose/info -e -n 1 2>/dev/null | grep -aA25 'name: "slam_robot"' | grep -aE "position|x:|y:" | head -3

echo "DRIVING STRAIGHT..."
timeout 6 ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.0}}" -r 20 >/dev/null 2>&1
timeout 2 ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" -r 20 >/dev/null 2>&1
sleep 1

echo "=== odom AFTER ==="
timeout 6 ros2 topic echo /odom --field pose.pose.position --once 2>/dev/null | head -3
echo "=== gz AFTER ==="
timeout 6 ign topic -t /world/slam_room/dynamic_pose/info -e -n 1 2>/dev/null | grep -aA25 'name: "slam_robot"' | grep -aE "position|x:|y:" | head -3
