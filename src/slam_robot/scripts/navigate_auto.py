#!/usr/bin/env python3
"""
navigate_auto.py — Automated Navigation for SLAM Robot (Nav2 Simple Commander)

Usage:
  python3 navigate_auto.py home          # Return to home position (0.0, 0.0)
  python3 navigate_auto.py patrol        # Automatically visit multiple locations and return
  python3 navigate_auto.py <x> <y>       # Navigate to specific (x, y) coordinates
"""

import sys
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def create_pose(navigator, x, y, yaw=0.0):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0
    # simplified orientation for 2D yaw: w=1.0 for yaw=0
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = 0.0
    pose.pose.orientation.w = 1.0
    return pose


def main():
    rclpy.init()
    navigator = BasicNavigator()

    # Wait for Nav2 to become active
    print("Nav2 stack ke active hone ka wait kar rahe hain...")
    navigator.waitUntilNav2Active()
    print("✓ Nav2 Active ho gaya!")

    args = sys.argv[1:]
    mode = args[0] if args else "home"

    if mode == "home":
        print("🎯 Goal: Returning to Home (x=0.0, y=0.0)...")
        home_pose = create_pose(navigator, 0.0, 0.0)
        navigator.goToPose(home_pose)

        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            if feedback:
                print(f"Distance remaining: {feedback.distance_remaining:.2f} m", end="\r")

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print("\n🎉 Successfully arrived at Home position!")
        elif result == TaskResult.CANCELED:
            print("\n⚠️ Navigation canceled!")
        elif result == TaskResult.FAILED:
            print("\n❌ Navigation failed!")

    elif mode == "patrol":
        print("🔄 Patrol Mode: Visiting waypoints...")
        waypoints = [
            create_pose(navigator, 2.0, 1.0),
            create_pose(navigator, 3.0, -1.0),
            create_pose(navigator, 1.0, -2.0),
            create_pose(navigator, 0.0, 0.0)  # Back to home
        ]
        navigator.followWaypoints(waypoints)

        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            if feedback:
                print(f"Executing waypoint: {feedback.current_waypoint + 1}/{len(waypoints)}", end="\r")

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print("\n🎉 Patrol complete! Robot returned to start position.")
        else:
            print(f"\n⚠️ Patrol finished with status: {result}")

    elif len(args) >= 2:
        target_x = float(args[0])
        target_y = float(args[1])
        print(f"🎯 Goal: Navigating to (x={target_x}, y={target_y})...")
        goal_pose = create_pose(navigator, target_x, target_y)
        navigator.goToPose(goal_pose)

        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            if feedback:
                print(f"Distance remaining: {feedback.distance_remaining:.2f} m", end="\r")

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f"\n🎉 Arrived at ({target_x}, {target_y})!")
        else:
            print(f"\n⚠️ Navigation ended with status: {result}")

    else:
        print("Invalid arguments. Use: home, patrol, or <x> <y>")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
