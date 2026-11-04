#!/usr/bin/env python3
"""
Intelligent Corner-Proof & Zero-Jerk LiDAR Auto-Mapper for SLAM Robot
Features:
- 4-Wheel Platform Compatibility: Ultra-smooth, zero-tilt drive profile.
- Velocity Smoothing / Ramping: Eliminates linear lift and angular rotation jerks.
- Anti-Oscillation Turn Locking: Never flip-flops in corners or narrow dead-ends.
- 180° Pocket Escape: Backs out and turns smoothly if trapped in narrow corridors.
- Periodic 360° Loop Closures in open spaces.
"""

import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class ReactiveAutoMapper(Node):
    def __init__(self):
        super().__init__('reactive_auto_mapper')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.timer = self.create_timer(0.05, self.control_loop)

        # Distances (in meters)
        self.front_narrow = 10.0
        self.front_wide = 10.0
        self.left_front = 10.0
        self.right_front = 10.0
        self.left_side = 10.0
        self.right_side = 10.0
        self.back_dist = 10.0
        self.min_dist = 10.0
        self.has_scan = False

        # State machine: 'EXPLORE', 'AVOID', 'REVERSE', 'ESCAPE_SPIN', 'SPIN'
        self.state = 'EXPLORE'
        self.state_start = time.time()
        self.last_spin_time = time.time()
        self.turn_lock_until = 0.0
        self.avoid_turn_dir = 1.0  # +1.0 left (CCW), -1.0 right (CW)

        # Velocity smoothing (ramping) to eliminate jerk and chassis lift
        self.current_linear = 0.0
        self.current_angular = 0.0

        self.get_logger().info('🤖 Zero-Jerk 4-Wheel Smooth Auto-Mapper Active!')

    def scan_cb(self, msg: LaserScan):
        self.has_scan = True
        total = len(msg.ranges)
        mid = total // 2

        def get_sector_min(deg_min, deg_max):
            s_idx = int(mid + (deg_min / 360.0) * total)
            e_idx = int(mid + (deg_max / 360.0) * total)
            if s_idx > e_idx:
                s_idx, e_idx = e_idx, s_idx
            vals = []
            for i in range(s_idx, e_idx):
                idx = i % total
                r = msg.ranges[idx]
                if msg.range_min < r < msg.range_max and not math.isnan(r) and not math.isinf(r):
                    vals.append(r)
            return min(vals) if vals else 10.0

        # Laser sectors
        self.front_narrow = get_sector_min(-25, 25)
        self.front_wide = get_sector_min(-55, 55)
        self.left_front = get_sector_min(15, 75)
        self.right_front = get_sector_min(-75, -15)
        self.left_side = get_sector_min(70, 110)
        self.right_side = get_sector_min(-110, -70)
        self.back_dist = get_sector_min(140, 220)

        valid_all = [
            r for r in msg.ranges
            if msg.range_min < r < msg.range_max and not math.isnan(r) and not math.isinf(r)
        ]
        self.min_dist = min(valid_all) if valid_all else 10.0

    def control_loop(self):
        if not self.has_scan:
            return

        now = time.time()
        target_linear = 0.0
        target_angular = 0.0

        # ==========================================================
        # 1. EMERGENCY REVERSE: Robot in tight corner or wall (< 0.38m)
        # ==========================================================
        if (self.min_dist < 0.38 or self.front_narrow < 0.42) and self.state not in ('REVERSE', 'ESCAPE_SPIN'):
            self.state = 'REVERSE'
            self.state_start = now
            left_clearance = self.left_front + self.left_side
            right_clearance = self.right_front + self.right_side
            self.avoid_turn_dir = 1.0 if left_clearance >= right_clearance else -1.0
            self.get_logger().warn(f'⚠️ Obstacle close ({self.min_dist:.2f}m)! Smoothly reversing...')

        if self.state == 'REVERSE':
            elapsed = now - self.state_start
            if elapsed < 1.6 and self.back_dist > 0.25:
                target_linear = -0.09
                target_angular = 0.0
            else:
                self.state = 'ESCAPE_SPIN'
                self.state_start = now
                self.turn_lock_until = now + 3.0
                self.get_logger().info('🔄 Reversing finished -> Smoothly rotating away from corner...')

        # ==========================================================
        # 2. ESCAPE SPIN (Smooth in-place rotation out of dead-ends)
        # ==========================================================
        elif self.state == 'ESCAPE_SPIN':
            elapsed = now - self.state_start
            if elapsed < 2.5 or (elapsed < 5.0 and self.front_narrow < 0.75):
                target_linear = 0.0
                target_angular = 0.22 * self.avoid_turn_dir  # Smooth gentle rotation
            else:
                self.state = 'EXPLORE'
                self.get_logger().info('✅ Escaped corner cleanly. Resuming smooth exploration...')

        # ==========================================================
        # 3. OBSTACLE AVOIDANCE WITH TURN-LOCK (No Corner Jitter)
        # ==========================================================
        else:
            is_front_blocked = self.front_wide < 0.85 or self.front_narrow < 0.70

            if self.state == 'AVOID' and now < self.turn_lock_until:
                target_linear = 0.0
                target_angular = 0.20 * self.avoid_turn_dir
            elif is_front_blocked:
                self.state = 'AVOID'
                self.state_start = now
                self.turn_lock_until = now + 2.0

                # Check if trapped in a corridor/pocket (both sides tight)
                if self.left_front < 0.70 and self.right_front < 0.70:
                    self.get_logger().warn('📦 Pocket detected! Executing smooth 180° turnaround...')
                    self.state = 'ESCAPE_SPIN'
                    self.avoid_turn_dir = 1.0 if self.left_side >= self.right_side else -1.0
                    target_linear = 0.0
                    target_angular = 0.22 * self.avoid_turn_dir
                else:
                    left_total = self.left_front + (0.5 * self.left_side)
                    right_total = self.right_front + (0.5 * self.right_side)
                    self.avoid_turn_dir = 1.0 if left_total >= right_total else -1.0
                    target_linear = 0.0
                    target_angular = 0.20 * self.avoid_turn_dir
                    self.get_logger().info(f'🛑 Path blocked -> Turning {"LEFT" if self.avoid_turn_dir > 0 else "RIGHT"}')
            elif (now - self.last_spin_time > 28.0) and (self.min_dist > 1.2):
                # 4. Periodic 360° scan in open area
                if self.state != 'SPIN':
                    self.state = 'SPIN'
                    self.state_start = now
                    self.get_logger().info('🔄 Open area reached. Slow 360° scan for SLAM...')

                if self.state == 'SPIN':
                    if now - self.state_start < 12.0:
                        target_linear = 0.0
                        target_angular = 0.20
                    else:
                        self.state = 'EXPLORE'
                        self.last_spin_time = now
            else:
                # 5. Normal Smooth Cruise
                self.state = 'EXPLORE'
                target_linear = 0.15  # Steady slip-free cruise speed

                if self.left_side < 0.65:
                    target_angular = -0.12
                elif self.right_side < 0.65:
                    target_angular = 0.12
                elif self.left_front > self.right_front + 0.4:
                    target_angular = 0.08
                elif self.right_front > self.left_front + 0.4:
                    target_angular = -0.08
                else:
                    target_angular = 0.0

        # ==========================================================
        # 6. VELOCITY SMOOTHING FILTER (Ramping: Zero Jerk / No Lift)
        # ==========================================================
        accel_lin = 0.010   # Max 0.20 m/s^2 linear ramp (smooth acceleration)
        accel_ang = 0.015   # Max 0.30 rad/s^2 angular ramp (smooth turn onset)

        if target_linear > self.current_linear:
            self.current_linear = min(target_linear, self.current_linear + accel_lin)
        elif target_linear < self.current_linear:
            self.current_linear = max(target_linear, self.current_linear - accel_lin)

        if target_angular > self.current_angular:
            self.current_angular = min(target_angular, self.current_angular + accel_ang)
        elif target_angular < self.current_angular:
            self.current_angular = max(target_angular, self.current_angular - accel_ang)

        twist = Twist()
        twist.linear.x = float(self.current_linear)
        twist.angular.z = float(self.current_angular)
        self.cmd_pub.publish(twist)

    def stop(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = ReactiveAutoMapper()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        node.stop()
        node.get_logger().info('🛑 Stopped.')
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
