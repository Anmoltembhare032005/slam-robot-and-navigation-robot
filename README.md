# 🤖 Autonomous SLAM & Navigation Mobile Robot

[![ROS 2](https://img.shields.io/badge/ROS_2-Humble-3498DB?style=for-the-badge&logo=ros)](https://docs.ros.org/en/humble/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Fortress-FF6F00?style=for-the-badge&logo=gazebo)](https://gazebosim.org/)
[![SLAM Toolbox](https://img.shields.io/badge/SLAM-Toolbox-27AE60?style=for-the-badge)](https://github.com/SteveMacenski/slam_toolbox)
[![Nav2](https://img.shields.io/badge/Navigation-Nav2-9B59B6?style=for-the-badge)](https://navigation.ros.org/)
[![Python](https://img.shields.io/badge/Python-3.10-3776AB?style=for-the-badge&logo=python&logoColor=white)](https://python.org)
[![OS](https://img.shields.io/badge/Ubuntu-22.04_LTS-E95420?style=for-the-badge&logo=ubuntu&logoColor=white)](https://releases.ubuntu.com/22.04/)

A comprehensive, end-to-end **ROS 2 Humble** and **Ignition Gazebo Fortress** robotics project featuring a 2-wheel differential-drive mobile robot equipped with **2D LiDAR** and **IMU**. The robot performs real-time simultaneous localization and mapping (SLAM) using **SLAM Toolbox**, saves high-resolution occupancy grid maps, and navigates autonomously in complex indoor environments using the **Nav2** stack.

---

## 📸 Visual Demos & Simulation Previews

### 🎥 Autonomous Navigation Demo (Live Action)
![Autonomous Navigation Demo](docs/images/navigation_demo.gif)
> 📹 *Nav2 stack planning dynamic paths, avoiding obstacles, and driving to designated goal poses.*  
> *(Full resolution video available at [`docs/videos/navigation_demo.webm`](docs/videos/navigation_demo.webm))*

---

### 🖼️ Simulation & Visualization

| 🌐 Ignition Gazebo Fortress Simulation | 🗺️ Live SLAM Mapping in RViz2 |
| :---: | :---: |
| ![Gazebo Simulation](docs/images/gazebo_simulation.png) | ![SLAM Map Making](docs/images/slam_map_making.png) |
| *Physics environment with 10x8m room & obstacles* | *SLAM Toolbox online async real-time mapping* |

| 🧭 Nav2 Global & Local Costmaps (RViz2) | 📄 Generated 2D Occupancy Grid Map |
| :---: | :---: |
| ![RViz Navigation](docs/images/rviz_visualization.png) | ![Occupancy Map](docs/images/slam_map.png) |
| *AMCL localization, costmaps & DWB local planner* | *Final 0.05m/pixel resolution saved map* |

---

## 🌟 Key Features

- **🤖 Differential-Drive Robot URDF/Xacro**: Custom lightweight mobile base with dual drive wheels, front/rear caster wheels, 360° 2D LiDAR, and 6-DOF IMU sensor.
- **🗺️ Real-Time SLAM (SLAM Toolbox)**: Online asynchronous LiDAR-based mapping, pose-graph optimization, and loop closure detection.
- **🧭 Nav2 Autonomous Navigation**: Full integration of Nav2 bringup, AMCL particle-filter localization, global path planning (NavFn), local trajectory generation (DWB Controller), and costmaps.
- **⚡ 1-Click Launch Scripts**: Pre-configured bash scripts for simulation startup, teleoperation, automated mapping, goal navigation, and clean shutdown.
- **🏢 Custom Gazebo Simulation World**: 10m x 8m enclosed indoor room (`slam_room.sdf`) populated with diverse static obstacles to test edge detection and path planning.
- **🎮 Autonomous & Teleop Modes**: Drive manually via keyboard (`teleop_twist_keyboard` / custom Python GUI) or let the robot map the area autonomously.

---

## 📐 System Architecture

```mermaid
flowchart TD
    subgraph Gazebo["Ignition Gazebo Fortress Simulation"]
        World["Closed World (slam_room.sdf)"]
        Robot["URDF Robot Model"]
        Lidar["2D GPU LiDAR Plugin"]
        IMU["IMU System Plugin"]
        DiffDrive["DiffDrive Plugin"]
    end

    subgraph Bridge["ros_gz_bridge (Topics & TF)"]
        GZ_Bridge["ROS 2 <-> Gazebo Bridge"]
    end

    subgraph ROS2["ROS 2 Humble Navigation Stack"]
        RSP["robot_state_publisher"]
        SLAM["SLAM Toolbox (online_async)"]
        Nav2["Nav2 Stack (AMCL + Costmaps + DWB)"]
        AutoMap["Auto Mapper / Teleop"]
    end

    subgraph Viz["Visualization & Control"]
        RViz["RViz2 (Live Map & Costmaps)"]
        User["User Goal / Terminal Action"]
    end

    World --> Robot
    Robot --> Lidar & IMU & DiffDrive
    Lidar & IMU & DiffDrive <--> GZ_Bridge
    GZ_Bridge <--> RSP & SLAM & Nav2
    AutoMap -->|/cmd_vel| DiffDrive
    SLAM -->|/map -> /odom TF| Nav2
    Nav2 -->|/cmd_vel| DiffDrive
    SLAM & Nav2 --> RViz
    User -->|2D Nav Goal| Nav2
```

---

## 📁 Repository Structure

```
slam_ws/
├── launch_sim.sh          # 🚀 1-Click: Launch Gazebo world + spawn robot + bridge
├── launch_slam.sh         # 🗺️ 1-Click: Launch SLAM Toolbox & RViz mapping UI
├── launch_nav.sh          # 🧭 1-Click: Launch Nav2 stack & navigation RViz
├── auto_map.sh            # 🤖 1-Click: Autonomous exploration & map builder
├── auto_nav.sh            # 🎯 1-Click: Autonomous multi-waypoint navigation demo
├── teleop.sh              # 🎮 1-Click: Keyboard teleoperation controller
├── save_map.sh            # 💾 1-Click: Save occupancy map (.yaml, .pgm, .png)
├── kill_all.sh            # 🛑 1-Click: Safe shutdown and DDS cleanup
├── docs/
│   ├── images/
│   │   ├── gazebo_simulation.png  # Gazebo fortress world preview
│   │   ├── rviz_visualization.png # RViz2 costmaps and AMCL visualization
│   │   ├── slam_map_making.png    # Live SLAM mapping preview
│   │   ├── navigation_demo.gif    # Animated navigation showcase
│   │   └── slam_map.png           # 2D occupancy grid map
│   └── videos/
│       └── navigation_demo.webm   # HD navigation screen recording
├── tools/                 # Diagnostic, alignment, and testing scripts
└── src/
    └── slam_robot/
        ├── CMakeLists.txt
        ├── package.xml
        ├── urdf/
        │   └── robot.urdf.xacro      # Robot kinematics, sensors, Gazebo plugins
        ├── worlds/
        │   └── slam_room.sdf         # 10x8 m enclosed room with obstacles
        ├── config/
        │   ├── slam_params.yaml      # SLAM Toolbox online_async config
        │   ├── nav2_params.yaml      # Nav2 costmaps, AMCL, and controller params
        │   └── fastrtps_udp.xml      # FastDDS UDP profile (prevents SHM locks)
        ├── launch/
        │   ├── sim.launch.py         # Gazebo simulation & ROS-GZ bridges
        │   ├── slam.launch.py        # SLAM Toolbox & mapping visualization
        │   └── nav.launch.py         # Nav2 lifecycle nodes & navigation RViz
        ├── maps/
        │   ├── slam_map.yaml         # Saved map metadata
        │   ├── slam_map.pgm          # PGM grayscale occupancy grid
        │   ├── slam_map.png          # PNG preview
        │   └── slam_posegraph.*      # Serialized SLAM pose graph
        ├── rviz/
        │   ├── slam.rviz             # RViz configuration for SLAM
        │   └── nav.rviz              # RViz configuration for Navigation
        └── scripts/
            ├── keyboard_control.py   # Keyboard teleop script
            ├── safe_auto_mapper.py   # LiDAR-based obstacle avoidance mapper
            └── navigate_auto.py      # Automated pose goal sender
```

---

## 🛠️ Prerequisites & Installation

### 1. Requirements
- **Ubuntu 22.04 LTS**
- **ROS 2 Humble Hawksbill** (Desktop Install)
- **Ignition Gazebo Fortress** (`ros-humble-ros-gz` & `ignition-fortress`)

### 2. Install Required ROS 2 Dependencies

```bash
sudo apt update
sudo apt install -y \
  ros-humble-slam-toolbox \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-ros-gz \
  ros-humble-teleop-twist-keyboard \
  ros-humble-robot-state-publisher \
  ros-humble-xacro
```

### 3. Clone & Build the Workspace

```bash
# Clone the repository
git clone git@github.com:Anmoltembhare032005/slam-robot-and-navigation-robot.git slam_ws
cd slam_ws

# Build the ROS 2 package
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# Source workspace
source install/setup.bash
```

---

## 🚀 Quick Start Guide

### Method A: Using 1-Click Convenience Scripts (Recommended)

| Step | Action | Script Command | Description |
|:---:|:---|:---|:---|
| **1** | **Simulation** | `./launch_sim.sh` | Launches Gazebo world, spawns the robot, and connects bridges |
| **2** | **SLAM Mapping** | `./launch_slam.sh` | Launches SLAM Toolbox and opens RViz2 mapping view |
| **3** | **Drive Robot** | `./teleop.sh` *or* `./auto_map.sh` | Drive manually via keyboard or run auto-exploration |
| **4** | **Save Map** | `./save_map.sh` | Saves generated map and posegraph into `src/slam_robot/maps/` |
| **5** | **Navigation** | `./launch_nav.sh` | Stops SLAM, loads saved map, and activates Nav2 + AMCL |
| **—** | **Shutdown** | `./kill_all.sh` | Safely terminates all nodes, Gazebo, and cleans DDS locks |

---

### Method B: Manual Step-by-Step Execution

#### 1️⃣ Start Gazebo Simulation (Terminal 1)
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch slam_robot sim.launch.py
```
> Starts Gazebo Fortress, spawns `slam_robot` at the origin `(0, 0, 0)`, and bridges `/scan`, `/odom`, `/imu`, `/tf`, and `/cmd_vel`.

#### 2️⃣ Start SLAM Toolbox (Terminal 2)
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch slam_robot slam.launch.py
```
> Runs `slam_toolbox` in `online_async` mode and displays the live expanding map in RViz2.

#### 3️⃣ Drive to Map the Environment (Terminal 3)
```bash
# Manual Teleoperation
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# Keys: i (forward), j (left), l (right), , (backward), k (stop)

# OR Run Autonomous Safe Auto Mapper
python3 src/slam_robot/scripts/safe_auto_mapper.py
```

#### 4️⃣ Save the Occupancy Map
```bash
# Save Occupancy Grid Map
ros2 run nav2_map_server map_saver_cli -f src/slam_robot/maps/slam_map

# Serialize Pose Graph for continuing SLAM later
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '$(pwd)/src/slam_robot/maps/slam_posegraph'}"
```

#### 5️⃣ Autonomous Navigation with Nav2 (Terminal 2)
Stop SLAM (Ctrl+C in Terminal 2), then launch Nav2:
```bash
ros2 launch slam_robot nav.launch.py
```
- In **RViz2**:
  - The robot initializes with AMCL (`set_initial_pose: true`).
  - Click **2D Goal Pose** anywhere in free space.
  - The global planner generates a path and DWB controller drives the robot while avoiding obstacles.
- Via **Terminal Command**:
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 2.5, y: 1.2}, orientation: {w: 1.0}}}}"
```

---

## 📡 ROS 2 Topics & TF Frame Hierarchy

### Core Topics:
| Topic | Type | Description |
|:---|:---|:---|
| `/scan` | `sensor_msgs/msg/LaserScan` | 2D LiDAR range data (~10 Hz) |
| `/odom` | `nav_msgs/msg/Odometry` | Wheel odometry computed by diff-drive plugin (~100 Hz) |
| `/imu` | `sensor_msgs/msg/Imu` | 6-axis IMU acceleration and angular velocity (~50 Hz) |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Velocity command sent to robot wheels |
| `/map` | `nav_msgs/msg/OccupancyGrid` | 2D occupancy grid generated by SLAM / Map Server |
| `/tf` | `tf2_msgs/msg/TFMessage` | Dynamic coordinate transformations |

### Coordinate Frames (TF Tree):
```
map  ──[ slam_toolbox / amcl ]──>  odom  ──[ gazebo diff_drive ]──>  base_link
                                                                          ├── lidar_link
                                                                          ├── imu_link
                                                                          ├── left_wheel
                                                                          └── right_wheel
```

---

## 🔍 Diagnostics & Sanity Checks

Verify data streams anytime during execution:
```bash
# Check sensor publish rates
ros2 topic hz /scan      # Expected: ~10 Hz
ros2 topic hz /odom      # Expected: ~100 Hz
ros2 topic hz /imu       # Expected: ~50 Hz

# Inspect TF frame hierarchy
ros2 run tf2_tools view_frames

# Check Nav2 Lifecycle Status
ros2 lifecycle get /amcl
```

---

## 👤 Author

- **Anmol Tembhare**
- GitHub: [@Anmoltembhare032005](https://github.com/Anmoltembhare032005)
- Email: [anmoltembhare032005@gmail.com](mailto:anmoltembhare032005@gmail.com)

---

## 📄 License

This project is open-source and available under the [MIT License](LICENSE).
