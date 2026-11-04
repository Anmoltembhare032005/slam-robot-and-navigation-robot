# SLAM & Navigation Benchmarking & Testing Tools

These scripts are helper utilities for measuring robot performance, odometry accuracy, and open-loop drive patterns:

- `odom_acc_test.sh`: Drives the robot straight for 6 seconds at 0.3 m/s and compares wheel odometry (`/odom`) against ground-truth pose from Ignition Gazebo.
- `drive_nav_test.sh`: Sends a test navigation goal to Nav2 action server and periodically logs `/odom` position.
- `coverage_drive.sh`: Systematic open-loop drive pattern (loops + arcs) for manual SLAM testing.
- `gentle_drive.sh`: Low-speed straight legs with slow 90-degree turns to minimize wheel slip during SLAM mapping.
