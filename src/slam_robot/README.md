# SLAM Robot — Ignition Gazebo Fortress + ROS 2 Humble (Simulation)

Differential-drive robot with a 2D Lidar + IMU, slam_toolbox mapping, and Nav2 navigation,
all simulated in Ignition Gazebo Fortress.

## Package layout

```
~/slam_ws/src/slam_robot/
├── urdf/robot.urdf.xacro      # diff-drive robot: lidar, IMU, gz plugins
├── worlds/slam_room.sdf       # closed room 10x8 m + 5 obstacles
├── launch/
│   ├── sim.launch.py          # gazebo + spawn + robot_state_publisher + bridge
│   ├── slam.launch.py         # slam_toolbox (online_async) + RViz
│   └── nav.launch.py          # nav2_bringup (AMCL) + RViz
├── config/
│   ├── slam_params.yaml       # slam_toolbox params
│   ├── nav2_params.yaml       # Nav2 params (footprint, DWB, AMCL)
│   └── fastrtps_udp.xml       # disables FastDDS SHM (needed in containers)
├── rviz/slam.rviz, nav.rviz
└── maps/slam_map.{pgm,yaml}   # saved map + slam_posegraph.{posegraph,data}
```

## Environment setup (every new terminal)

```bash
source /opt/ros/humble/setup.bash
source ~/slam_ws/install/setup.bash
cd ~/slam_ws
```

## Run order

### 1. Simulation (Terminal 1)

```bash
ros2 launch slam_robot sim.launch.py
```

Starts Ignition Gazebo with the closed room, spawns the robot (x=y=z=0), runs
`robot_state_publisher`, and bridges `/clock`, `/cmd_vel`, `/odom`, `/tf`, `/scan`,
`/imu`, `/joint_states` (GZ <-> ROS).

### 2. Mapping with slam_toolbox (Terminal 2)

```bash
ros2 launch slam_robot slam.launch.py
# or without RViz in headless environments
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=~/slam_ws/install/slam_robot/share/slam_robot/config/slam_params.yaml \
  use_sim_time:=true
```

Publishes `map -> odom` TF and `/map`.

### 3. Drive manually (Terminal 3)

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# keys: i=forward, ,=back, j=turn left, l=turn right, k=stop
```

Drive SLOWLY around the room several times (straight legs + turns) so the walls
are observed; watch the map grow in RViz / `ros2 topic echo /map --field info --once`.
Drive straight and turn at <= 0.3 m/s to avoid wheel slip during turns.

### 4. Save the map (while slam_toolbox is still running)

```bash
# occupancy map (.png written by Humble; convert to .pgm or keep yaml as-is)
ros2 run nav2_map_server map_saver_cli -f ~/slam_ws/src/slam_robot/maps/slam_map

# pose graph serialization
ros2 service call /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph \
  "{filename: /home/anmol/slam_ws/src/slam_robot/maps/slam_posegraph}"

# optional PGM conversion
python3 -c "from PIL import Image; Image.open('~/slam_ws/src/slam_robot/maps/slam_map.png').save('~/slam_ws/src/slam_robot/maps/slam_map.pgm', format='PPM')"
# then edit slam_map.yaml: image: slam_map.pgm
```

Stop the SLAM stack (Ctrl+C in Terminal 2) before starting navigation.

### 5. Navigation with Nav2 + AMCL (Terminal 2 again)

```bash
ros2 launch slam_robot nav.launch.py
```

In RViz:
- already localizing (`set_initial_pose: true` at map origin). If the pose drifts, use
  **2D Pose Estimate** on the robot's true position.
- click **2D Goal Pose** anywhere in free space -> the robot plans and drives there.

Terminal check:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 2.5, y: 1.2}, orientation: {w: 1.0}}}}"
```

### Kill everything

```bash
~/slam_ws/kill_all.sh    # stops sim, SLAM, Nav2, RViz, bridges; cleans DDS locks
```

## Useful sanity checks

```bash
ros2 topic hz /scan                  # ~10 Hz
ros2 topic hz /odom                  # ~100 Hz
ros2 topic hz /imu                   # ~50 Hz
ros2 run tf2_tools view_frames       # map -> odom -> base_link -> lidar_link
ros2 lifecycle get /amcl             # active [3] after nav launch
```

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `ign topic -l` shows only `/cmd_vel`, no `/scan` | robot lost DSmid sensors or spawn failed | verify server alive, re-run `kill_all.sh`, relaunch sim; check `~/.ros/log` |
| robot does not move on cmd_vel | obstacles or missed spawn препятствия | ensure clean spawn (robots at origin); check `kill_all.sh` then fresh boot |
| `Failed init_port fastrtps_portXXXX` spam | FastDDS SHM locks in /dev/shm | `rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_port*` (kill_all.sh does this) |
| SLAM map has walls в wrong places / smudges | odometry drift from slip turning too fast / collisions | revert and re-drive slowly; restart SLAM fresh |
| Nav2 "Failed to bring up" one-time lifecycle race | container heavy load | re-run nav launch after clean state (works when started fresh) |
| Planning fails to valid goal | the map missing walls (mapping too short) | map again longer (few laps around the room); re-save map |
| BT error "Node not recognized" | missing plugin in nav2_params.yaml | needed libs listed under `plugin_lib_names` — keep names matching your nav2 build |
| lidar frame TF missing | bridge split | sim.launch.py already bridges /tf with GZ->ROS direction |
| stale "Moved backwards in time" warnings | sim clock restarted after a relaunch | restart RSP or the whole tree |

## Known design notes

- Fortress needs world-level plugin `ignition-gazebo-imu-system` for the IMU sensor.
- DiffDrive listens on absolute `/cmd_vel` (<topic>/cmd_vel</topic> in the URDF plugin).
- gpu_lidar requires the world Sensors system with `ogre2`.
- FastDDS shared-memory transport disabled via `FASTRTPS_DEFAULT_PROFILES_FILE`; UDPv4 only.
- velocity_smoother expects vector params in Humble: `max_velocity: [x, y, theta]`.
