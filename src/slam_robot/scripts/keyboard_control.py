#!/usr/bin/env python3
"""
Smooth Constant-Speed Keyboard Controller for SLAM Robot with Collision / Slip Guard
Controls:
  [ ↑ ] / [ w ] : Move Forward  (Constant Speed)
  [ ↓ ] / [ s ] : Move Backward (Constant Speed)
  [ ← ] / [ a ] : Turn Left     (Constant Speed)
  [ → ] / [ d ] : Turn Right    (Constant Speed)
  [ Space ]     : Emergency Stop
  [ b ]         : Toggle Collision / Slip Guard (ON / OFF)
  [ q ]         : Quit
"""

import sys
import math
import select
import termios
import tty
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

BANNER = """
==========================================================
    🎮 SLAM ROBOT — KEYBOARD CONTROL WITH SLIP GUARD
==========================================================
  Forward  :  [ ↑ ] or [ w ]  (Linear:  +0.15 m/s)
  Backward :  [ ↓ ] or [ s ]  (Linear:  -0.10 m/s)
  Turn Left:  [ ← ] or [ a ]  (Angular: +0.22 rad/s)
  Turn Right: [ → ] or [ d ]  (Angular: -0.22 rad/s)
  Stop     :  [ Space ] or [ x ]
  Guard Tog:  [ b ] (Toggle Collision & Slip Guard)
  Quit     :  [ q ] or [ Ctrl+C ]
==========================================================
"""

# Constant speed settings for zero-slip, crystal-clear SLAM
CONST_LINEAR_FWD  = 0.15   # m/s
CONST_LINEAR_REV  = -0.10  # m/s
CONST_ANGULAR     = 0.22   # rad/s

# Safe distance thresholds (LiDAR to obstacle)
# LiDAR is at x = +0.10m, front chassis edge is at x = +0.20m (0.10m ahead of lidar)
FRONT_STOP_DIST   = 0.22   # Stop forward when obstacle is within 22cm of lidar (12cm from chassis)
REAR_STOP_DIST    = 0.25   # Stop backward when obstacle is within 25cm of rear

# Auto-stop timeout if no key pressed (in seconds)
KEY_TIMEOUT = 0.45


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_control')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.timer = self.create_timer(0.05, self.publish_loop)  # 20 Hz

        self.target_linear = 0.0
        self.target_angular = 0.0
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.last_key_time = time.time()

        # Collision & Slip Guard state
        self.slip_guard_enabled = True
        self.front_obstacle_dist = 10.0
        self.rear_obstacle_dist = 10.0
        self.front_blocked = False
        self.rear_blocked = False

        print(BANNER)
        print("🛡️  Collision / Slip Guard: [ ENABLED ] (Press 'b' to toggle)")
        print("Ready! Press Arrow keys or WASD to drive...\n")

    def scan_callback(self, msg: LaserScan):
        total = len(msg.ranges)
        if total == 0:
            return

        mid = total // 2  # Front center is at index mid (0 rad relative to robot)

        # Helper to get minimum distance in a given angular range (degrees)
        def get_min_dist(deg_start, deg_end):
            start_idx = int(mid + (deg_start / 360.0) * total)
            end_idx = int(mid + (deg_end / 360.0) * total)
            if start_idx > end_idx:
                start_idx, end_idx = end_idx, start_idx
            
            valid = []
            for i in range(start_idx, end_idx):
                idx = i % total
                r = msg.ranges[idx]
                if msg.range_min < r < msg.range_max and not math.isnan(r) and not math.isinf(r):
                    valid.append(r)
            return min(valid) if valid else 10.0

        # Front cone: -30 deg to +30 deg
        self.front_obstacle_dist = get_min_dist(-30, 30)
        # Rear cone: 150 deg to 210 deg
        self.rear_obstacle_dist = get_min_dist(150, 210)

        self.front_blocked = (self.front_obstacle_dist < FRONT_STOP_DIST)
        self.rear_blocked = (self.rear_obstacle_dist < REAR_STOP_DIST)

    def toggle_guard(self):
        self.slip_guard_enabled = not self.slip_guard_enabled
        status = "ENABLED" if self.slip_guard_enabled else "DISABLED"
        print(f"\n🛡️  Collision / Slip Guard: [ {status} ]\n")

    def set_motion(self, linear, angular):
        # Slip Guard check
        if self.slip_guard_enabled:
            if linear > 0.0 and self.front_blocked:
                # Block forward movement to prevent wheel spinning against obstacle
                self.target_linear = 0.0
                self.target_angular = angular
                self.last_key_time = time.time()
                sys.stdout.write(f"\r⚠️  [SLIP GUARD] Front blocked ({self.front_obstacle_dist:.2f}m)! Reverse or turn to clear.   ")
                sys.stdout.flush()
                return
            elif linear < 0.0 and self.rear_blocked:
                self.target_linear = 0.0
                self.target_angular = angular
                self.last_key_time = time.time()
                sys.stdout.write(f"\r⚠️  [SLIP GUARD] Rear blocked ({self.rear_obstacle_dist:.2f}m)! Drive forward or turn.        ")
                sys.stdout.flush()
                return

        self.target_linear = linear
        self.target_angular = angular
        self.last_key_time = time.time()

    def publish_loop(self):
        # Auto stop if key was released (no fresh key press for KEY_TIMEOUT seconds)
        if time.time() - self.last_key_time > KEY_TIMEOUT:
            self.target_linear = 0.0
            self.target_angular = 0.0

        # Enforce Slip Guard dynamically even during ongoing ramp
        if self.slip_guard_enabled:
            if self.target_linear > 0.0 and self.front_blocked:
                self.target_linear = 0.0
            elif self.target_linear < 0.0 and self.rear_blocked:
                self.target_linear = 0.0

        # Gentle ramp filter (removes any start/stop jerk)
        accel_lin = 0.015  # 0.30 m/s^2
        accel_ang = 0.025  # 0.50 rad/s^2

        if self.target_linear > self.current_linear:
            self.current_linear = min(self.target_linear, self.current_linear + accel_lin)
        elif self.target_linear < self.current_linear:
            self.current_linear = max(self.target_linear, self.current_linear - accel_lin)

        if self.target_angular > self.current_angular:
            self.current_angular = min(self.target_angular, self.current_angular + accel_ang)
        elif self.target_angular < self.current_angular:
            self.current_angular = max(self.target_angular, self.current_angular - accel_ang)

        # Snap small values to zero
        if abs(self.current_linear) < 0.005:
            self.current_linear = 0.0
        if abs(self.current_angular) < 0.005:
            self.current_angular = 0.0

        msg = Twist()
        msg.linear.x = float(self.current_linear)
        msg.angular.z = float(self.current_angular)
        self.pub.publish(msg)

    def stop(self):
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.pub.publish(Twist())


def get_key(settings):
    """Read a single key or arrow escape sequence without blocking."""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
    key = ''
    if rlist:
        key = sys.stdin.read(1)
        # Check for escape sequence (Arrow keys)
        if key == '\x1b':
            # Non-blocking read of the next two chars
            r2, _, _ = select.select([sys.stdin], [], [], 0.02)
            if r2:
                key += sys.stdin.read(2)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = KeyboardTeleop()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            key = get_key(settings)

            if not key:
                continue

            # Arrow keys & WASD mapping
            if key in ('\x1b[A', 'w', 'W'):  # UP
                node.set_motion(CONST_LINEAR_FWD, 0.0)
                sys.stdout.write(f"\r🚗 FORWARD  | Lin: {CONST_LINEAR_FWD:+.2f} m/s | Ang:  0.00 rad/s   ")
                sys.stdout.flush()

            elif key in ('\x1b[B', 's', 'S'):  # DOWN
                node.set_motion(CONST_LINEAR_REV, 0.0)
                sys.stdout.write(f"\r🔙 BACKWARD | Lin: {CONST_LINEAR_REV:+.2f} m/s | Ang:  0.00 rad/s   ")
                sys.stdout.flush()

            elif key in ('\x1b[D', 'a', 'A'):  # LEFT
                node.set_motion(0.0, CONST_ANGULAR)
                sys.stdout.write(f"\r↺ TURN LEFT| Lin:  0.00 m/s | Ang: {CONST_ANGULAR:+.2f} rad/s   ")
                sys.stdout.flush()

            elif key in ('\x1b[C', 'd', 'D'):  # RIGHT
                node.set_motion(0.0, -CONST_ANGULAR)
                sys.stdout.write(f"\r↻ TURN RGHT| Lin:  0.00 m/s | Ang: {-CONST_ANGULAR:+.2f} rad/s   ")
                sys.stdout.flush()

            elif key in (' ', 'x', 'X'):  # STOP
                node.set_motion(0.0, 0.0)
                sys.stdout.write(f"\r🛑 STOPPED  | Lin:  0.00 m/s | Ang:  0.00 rad/s                 ")
                sys.stdout.flush()

            elif key in ('b', 'B'):  # TOGGLE SLIP GUARD
                node.toggle_guard()

            elif key in ('q', 'Q', '\x03'):  # QUIT (q or Ctrl+C)
                print("\n\nQuitting keyboard controller...")
                break

    except Exception as e:
        print(f"\nError: {e}")
    finally:
        node.stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()
        print("Keyboard controller closed cleanly.\n")


if __name__ == '__main__':
    main()
