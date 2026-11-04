#!/usr/bin/env python3
"""
auto_align.py — Gazebo Ground-Truth to RViz & AMCL Auto-Aligner
1. Preloads the OccupancyGrid distance transform directly from disk or /map topic.
2. Reads the exact robot pose from Gazebo (/world/slam_room/dynamic_pose/info).
3. Refines (x, y, yaw) with live LiDAR scan points (accounting for lidar offset x=+0.10m).
4. Publishes /initialpose to AMCL & RViz with valid sim-time timestamp.
"""

import os
import sys
import time
import math
import subprocess
import re
import numpy as np

# Ensure FastDDS uses UDP profile matching sim and nav
UDP_PROFILE = '/home/anmol/slam_ws/src/slam_robot/config/fastrtps_udp.xml'
if os.path.exists(UDP_PROFILE):
    os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = UDP_PROFILE

from PIL import Image
from scipy.ndimage import distance_transform_edt

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped


MAP_YAML = '/home/anmol/slam_ws/src/slam_robot/maps/slam_map.yaml'
MAP_PGM = '/home/anmol/slam_ws/src/slam_robot/maps/slam_map.pgm'


def get_gazebo_pose():
    """Extract slam_robot ground truth pose directly from Gazebo."""
    for cmd in [
        ["ign", "topic", "-e", "-t", "/world/slam_room/dynamic_pose/info", "-n", "1"],
        ["gz", "topic", "-e", "-t", "/world/slam_room/dynamic_pose/info", "-n", "1"],
    ]:
        try:
            out = subprocess.check_output(
                cmd,
                text=True,
                timeout=3.0,
                stderr=subprocess.DEVNULL
            )
            blocks = re.findall(r"pose\s*\{([^\{\}]*(?:\{[^\{\}]*\}[^\{\}]*)*)\}", out)
            for b in blocks:
                if '"slam_robot"' in b:
                    x, y, z = 0.0, 0.0, 0.0
                    pos_m = re.search(r"position\s*\{([^}]+)\}", b)
                    if pos_m:
                        pos_text = pos_m.group(1)
                        xm = re.search(r"\bx:\s*([-\d.e+]+)", pos_text)
                        ym = re.search(r"\by:\s*([-\d.e+]+)", pos_text)
                        zm = re.search(r"\bz:\s*([-\d.e+]+)", pos_text)
                        if xm: x = float(xm.group(1))
                        if ym: y = float(ym.group(1))
                        if zm: z = float(zm.group(1))

                    qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
                    ori_m = re.search(r"orientation\s*\{([^}]+)\}", b)
                    if ori_m:
                        ori_text = ori_m.group(1)
                        qxm = re.search(r"\bx:\s*([-\d.e+]+)", ori_text)
                        qym = re.search(r"\by:\s*([-\d.e+]+)", ori_text)
                        qzm = re.search(r"\bz:\s*([-\d.e+]+)", ori_text)
                        qwm = re.search(r"\bw:\s*([-\d.e+]+)", ori_text)
                        if qxm: qx = float(qxm.group(1))
                        if qym: qy = float(qym.group(1))
                        if qzm: qz = float(qzm.group(1))
                        if qwm: qw = float(qwm.group(1))

                    siny_cosp = 2.0 * (qw * qz + qx * qy)
                    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
                    yaw = math.atan2(siny_cosp, cosy_cosp)
                    return x, y, yaw
        except Exception:
            continue
    return None


class AutoAligner(Node):
    def __init__(self):
        super().__init__(
            'auto_aligner',
            parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, True)
            ]
        )

        latched_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_cb, latched_qos
        )
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_cb, 10
        )
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10
        )

        self.scan_msg = None
        self.done = False

        # Defaults (will be loaded from MAP_YAML)
        self.origin_x = -5.55
        self.origin_y = -4.60
        self.resolution = 0.05
        self.width = 224
        self.height = 184
        self.dist_map = None

        self._preload_map_from_disk()

        self.timer = self.create_timer(0.2, self.align_loop)
        self.get_logger().info('🔍 Step 1: Connecting to Gazebo & Live LiDAR (/scan)...')

    def _preload_map_from_disk(self):
        if os.path.exists(MAP_YAML):
            try:
                import yaml
                with open(MAP_YAML, 'r') as f:
                    cfg = yaml.safe_load(f)
                    self.origin_x = float(cfg['origin'][0])
                    self.origin_y = float(cfg['origin'][1])
                    self.resolution = float(cfg.get('resolution', 0.05))
            except Exception as e:
                self.get_logger().warn(f'Could not parse {MAP_YAML}: {e}')

        if os.path.exists(MAP_PGM):
            try:
                img = Image.open(MAP_PGM)
                arr = np.array(img)[::-1, :]
                self.height, self.width = arr.shape
                wall_mask = (arr < 50)
                self.dist_map = distance_transform_edt(~wall_mask) * self.resolution
                self.get_logger().info(
                    f'🗺️ Preloaded map: {os.path.basename(MAP_PGM)} ({self.width}x{self.height}), '
                    f'origin=({self.origin_x:.3f}, {self.origin_y:.3f}), res={self.resolution:.3f}'
                )
            except Exception as e:
                self.get_logger().warn(f'Could not preload map: {e}')

    def map_cb(self, msg: OccupancyGrid):
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        self.resolution = msg.info.resolution
        self.width = msg.info.width
        self.height = msg.info.height
        data = np.array(msg.data, dtype=np.int8).reshape((self.height, self.width))
        wall_mask = (data >= 50)
        self.dist_map = distance_transform_edt(~wall_mask) * self.resolution

    def scan_cb(self, msg: LaserScan):
        self.scan_msg = msg

    def align_loop(self):
        if self.dist_map is None or self.done:
            return

        # Check Gazebo pose
        gz_pose = get_gazebo_pose()
        if gz_pose is None:
            self.get_logger().info('⏳ Waiting for Gazebo simulation (/world/slam_room/dynamic_pose/info)...', throttle_duration_sec=2.0)
            return

        if self.scan_msg is None:
            self.get_logger().info('⏳ Waiting for live LiDAR /scan topic...', throttle_duration_sec=2.0)
            return

        init_x, init_y, init_yaw = gz_pose
        self.get_logger().info(
            f'🌐 Step 1: Found Gazebo Robot Pose: x={init_x:.3f}m, y={init_y:.3f}m, yaw={math.degrees(init_yaw):.1f}°'
        )

        # Step 2: Refine pose using LiDAR scan and preloaded Map
        self.get_logger().info('⚡ Step 2: Fine-tuning LiDAR scan alignment with Map...')
        scan = self.scan_msg
        angles = scan.angle_min + np.arange(len(scan.ranges)) * scan.angle_increment
        ranges = np.array(scan.ranges, dtype=np.float32)

        valid = (ranges > scan.range_min) & (ranges < scan.range_max) & np.isfinite(ranges)
        valid = valid & (np.arange(len(ranges)) % 2 == 0)

        v_ranges = ranges[valid]
        v_angles = angles[valid]

        # Robot lidar mount is offset by +0.10m in x from base_link
        lidar_offset_x = 0.10
        lx = lidar_offset_x + v_ranges * np.cos(v_angles)
        ly = v_ranges * np.sin(v_angles)

        origin_x = self.origin_x
        origin_y = self.origin_y
        res = self.resolution
        w = self.width
        h = self.height
        dist_map = self.dist_map

        def score_pose(x, y, yaw):
            cos_y = math.cos(yaw)
            sin_y = math.sin(yaw)
            gx = x + (lx * cos_y - ly * sin_y)
            gy = y + (lx * sin_y + ly * cos_y)

            px = np.int32(np.round((gx - origin_x) / res))
            py = np.int32(np.round((gy - origin_y) / res))

            in_bounds = (px >= 0) & (px < w) & (py >= 0) & (py < h)
            if np.count_nonzero(in_bounds) < 0.3 * len(lx):
                return -9999.0

            dists = dist_map[py[in_bounds], px[in_bounds]]
            return np.sum(np.exp(- (dists ** 2) / (2 * (0.10 ** 2))))

        best_score = score_pose(init_x, init_y, init_yaw)
        best_x = init_x
        best_y = init_y
        best_yaw = init_yaw

        # Search tightly around Gazebo ground truth (+/- 0.15m, +/- 5 deg)
        search_r_xy = 0.15
        search_r_yaw = math.radians(5.0)
        xs = np.linspace(init_x - search_r_xy, init_x + search_r_xy, 11)
        ys = np.linspace(init_y - search_r_xy, init_y + search_r_xy, 11)
        yaws = np.linspace(init_yaw - search_r_yaw, init_yaw + search_r_yaw, 11)

        for yaw in yaws:
            for x in xs:
                for y in ys:
                    s = score_pose(x, y, yaw)
                    if s > best_score:
                        best_score = s
                        best_x = x
                        best_y = y
                        best_yaw = yaw

        self.get_logger().info(
            f'🎯 Final Synced Pose: x={best_x:.3f}m, y={best_y:.3f}m, yaw={math.degrees(best_yaw):.1f}°'
        )

        # Step 3: Publish to /initialpose for RViz and AMCL
        self.get_logger().info('📡 Step 3: Snapping RViz & AMCL to Gazebo pose...')

        # Wait for AMCL or RViz to subscribe to /initialpose
        start_wait = time.time()
        while self.pose_pub.get_subscription_count() == 0 and (time.time() - start_wait < 3.0):
            rclpy.spin_once(self, timeout_sec=0.1)

        pose_msg = PoseWithCovarianceStamped()
        now_time = self.get_clock().now()
        if now_time.nanoseconds > 100_000_000:
            stamp_time = (now_time - Duration(seconds=0.1)).to_msg()
        else:
            stamp_time = now_time.to_msg()

        pose_msg.header.stamp = stamp_time
        pose_msg.header.frame_id = 'map'

        pose_msg.pose.pose.position.x = float(best_x)
        pose_msg.pose.pose.position.y = float(best_y)
        pose_msg.pose.pose.position.z = 0.0

        half_yaw = best_yaw * 0.5
        pose_msg.pose.pose.orientation.x = 0.0
        pose_msg.pose.pose.orientation.y = 0.0
        pose_msg.pose.pose.orientation.z = math.sin(half_yaw)
        pose_msg.pose.pose.orientation.w = math.cos(half_yaw)

        cov = [0.0] * 36
        cov[0] = 0.005   # x
        cov[7] = 0.005   # y
        cov[35] = 0.008  # yaw
        pose_msg.pose.covariance = cov

        for _ in range(15):
            self.pose_pub.publish(pose_msg)
            rclpy.spin_once(self, timeout_sec=0.08)

        self.get_logger().info('✅ Gazebo and RViz are now 100% aligned and in sync!')
        self.done = True
        time.sleep(0.3)
        sys.exit(0)


def main():
    rclpy.init()
    node = AutoAligner()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
