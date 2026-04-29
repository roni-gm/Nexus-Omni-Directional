# Nexus Omni 4WD - ROS2 Navigation System (A* + SLAM Toolbox + Pure Pursuit)

Sistem ini adalah implementasi lengkap navigasi robot omni 4WD berbasis **ROS 2 Humble** dengan simulasi **Gazebo Classic**, menggunakan pipeline custom:

* SLAM (slam_toolbox)
* Global Planner (A* Algorithm)
* Local Controller (Pure Pursuit + Obstacle Avoidance)
* RViz2 Visualization

---

# 1. Gambaran Sistem (Arsitektur)

```
                +-------------------+
                |   Gazebo World    |
                +-------------------+
                          |
                        LiDAR
                          |
                +-------------------+
                |  slam_toolbox     |
                |  (Mapping /map)   |
                +-------------------+
                          |
                          v
                +-------------------+
                |   A* Planner      |
                |   (/path)         |
                +-------------------+
                          |
                          v
                +---------------------------+
                |  Path Follower (Core)     |
                |  Pure Pursuit + Safety    |
                +---------------------------+
                          |
                          v
                    /cmd_vel (Robot)
```

---

# 2. Spesifikasi Sistem

## Robot

* Omni 4WD
* Radius robot: **0.16 m**
* Safety margin: **0.05 m**
* Inflation radius: **0.21 m**

## Map

* Resolution: **0.07 m/cell**
* Obstacle threshold: **65**
* Environment: Indoor 8m x 8m (Gazebo Classic)

---

# 3. Requirement System

## OS

* Ubuntu 22.04 / 24.04
* ROS 2 Humble
* Gazebo Classic (gazebo11)

## Install Dependencies

```bash
sudo apt update
sudo apt install -y \
ros-humble-rviz2 \
ros-humble-slam-toolbox \
ros-humble-navigation2 \
ros-humble-nav-msgs \
ros-humble-geometry-msgs
```

---

# 4. Workspace Setup

```bash
mkdir -p ~/astar_ws/src
cd ~/astar_ws/src
```

Clone project:

```bash
git clone <repo-kamu>
```

Build:

```bash
cd ~/astar_ws
colcon build --symlink-install
source install/setup.bash
```

---

# 5. Struktur Package

```
robot_controller/
 ├── astar_planner.py
 ├── path_follower.py
 ├── rviz.launch.py
gazebo_sim/
 ├── world/indoor_obstacles.world
 ├── launch/sim.launch.py
 ├── config/slam.yaml
```

---

# 6. Cara Menjalankan (MANUAL MODE)

## 6.1 Launch Gazebo

```bash
ros2 launch gazebo_sim sim.launch.py
```

## 6.2 Jalankan SLAM

```bash
ros2 launch slam_toolbox online_async_launch.py \
params_file:=~/astar_ws/src/gazebo_sim/config/slam.yaml
```

## 6.3 Jalankan RViz

```bash
ros2 launch robot_controller rviz.launch.py
```

## 6.4 Jalankan A* Planner

```bash
ros2 run robot_controller astar_planner
```

## 6.5 Jalankan Path Follower

```bash
ros2 run robot_controller path_follower
```

---

# 7. Cara Menggunakan Robot

1. Buka RViz2
2. Klik **2D Pose Estimate** (posisi awal robot)
3. Klik **2D Nav Goal** (1 atau 2 titik)
4. Sistem akan otomatis:

   * Generate map (SLAM)
   * Hitung path (A*)
   * Follow path (Pure Pursuit)
   * Stop di goal

---

# 8. AUTO LAUNCH (REKOMENDASI)

Buat file:

```
launch/full_system.launch.py
```

Isi:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        Node(
            package='gazebo_sim',
            executable='gazebo_world',
            output='screen'
        ),

        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            output='screen'
        ),

        Node(
            package='robot_controller',
            executable='astar_planner',
            output='screen'
        ),

        Node(
            package='robot_controller',
            executable='path_follower',
            output='screen'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            output='screen'
        ),
    ])
```

Run:

```bash
ros2 launch robot_controller full_system.launch.py
```

---

# 9. Parameter Tuning Penting

## Path Following

```python
LOOKAHEAD_DIST = 0.7
GOAL_REACH_DIST = 0.25
CMD_SMOOTHING = 0.2
```

## Obstacle Avoidance

```python
STOP_DISTANCE = 0.25
SLOW_DISTANCE = 0.5
CRITICAL_DISTANCE = 0.15
```

## Velocity

```python
MAX_LINEAR_VEL = 0.4
MAX_LATERAL_VEL = 0.35
MAX_ANGULAR_VEL = 0.8
```

---

# 10. Troubleshooting

## Robot tidak jalan setelah goal

* cek `/path` kosong atau tidak
* cek astar planner aktif

## Robot goyang di target

* naikkan GOAL_REACH_DIST → 0.25 - 0.3
* reset prev_cmd saat goal

## Gerakan patah-patah

* turunkan CMD_SMOOTHING → 0.1
* naikkan timer ke 0.02s

## Robot terlalu sensitif obstacle

* cek LiDAR noise
* kecilkan CRITICAL_DISTANCE

---

# 11. Optimasi Sistem

* Jalankan Gazebo dengan RTF > 0.9
* Pisahkan terminal (SLAM / planner / follower)
* Gunakan RViz tanpa plugin berat

---

# 12. Catatan Penting

Sistem ini **tidak menggunakan Nav2 penuh**, tetapi pipeline custom:

* SLAM Toolbox → Mapping
* A* → Global Planning
* Pure Pursuit → Local Control

Keuntungan:

* lebih ringan
* mudah dimodifikasi
* cocok untuk riset & skripsi

---

# Author

Nexus Omni Navigation System - ROS2 Humble + Gazebo Classic
