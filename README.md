# 🚀 Nexus Omni 4WD - ROS2 Humble Navigation System (SLAM + A* + Pure Pursuit)

## 📌 Project Overview

Project ini adalah simulasi sistem navigasi robot **omni 4WD** menggunakan **ROS 2 Humble** dan **Gazebo Classic** pada lingkungan indoor (8m x 8m). Sistem ini dirancang sebagai pipeline navigasi lengkap yang terdiri dari:

* **SLAM (slam_toolbox)** → pembuatan peta secara real-time
* **A* Path Planning** → perencanaan jalur global
* **Pure Pursuit Controller** → tracking jalur
* **Obstacle Avoidance berbasis LiDAR** → keamanan navigasi
* **RViz2 Visualization** → monitoring dan kontrol interaktif

🎯 Tujuan proyek ini adalah memberikan implementasi navigasi robot yang ringan, modular, dan mudah dipahami tanpa menggunakan stack Nav2 penuh.

---

# 🧠 System Architecture

```
Gazebo Classic (World Simulation)
        ↓
   LiDAR Sensor
        ↓
slam_toolbox (Mapping /map)
        ↓
   A* Planner (Global Path /path)
        ↓
 Path Follower Node
 (Pure Pursuit + Safety Layer)
        ↓
     /cmd_vel
        ↓
   Robot Omni 4WD
```

---

# 💻 System Requirements

## 🖥️ OS

* Ubuntu 22.04 LTS (recommended)

## 🤖 ROS2

* ROS2 Humble

## 🎮 Simulator

* Gazebo Classic (gazebo11)

## 📦 Dependencies

Install dependencies berikut:

```bash
sudo apt update
sudo apt install -y \
ros-humble-rviz2 \
ros-humble-slam-toolbox \
ros-humble-nav-msgs \
ros-humble-geometry-msgs \
ros-humble-tf2-ros \
python3-colcon-common-extensions \
git
```

---

# 📁 Project Structure (Assumption Standard ROS2 Workspace)

```
astar_ws/
 ├── src/
 │   ├── gazebo_sim/
 │   │    ├── world/
 │   │    ├── launch/
 │   │    ├── config/
 │   │
 │   ├── robot_controller/
 │        ├── astar_planner.py
 │        ├── path_follower.py
 │        ├── rviz.launch.py
```

---

# ⚙️ Installation Guide (Step-by-Step)

## 1. Buat Workspace ROS2

```bash
mkdir -p ~/astar_ws/src
cd ~/astar_ws/src
```

## 2. Clone Project

```bash
git clone <repo-anda>
```

## 3. Install Dependency ROS2

```bash
cd ~/astar_ws
rosdep install --from-paths src --ignore-src -r -y
```

## 4. Build Workspace

```bash
colcon build --symlink-install
```

## 5. Source Workspace

```bash
source install/setup.bash
```

👉 Tambahkan ke bashrc agar otomatis:

```bash
echo "source ~/astar_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

# 🚀 Cara Menjalankan Sistem

## 🧩 1. Jalankan Gazebo Simulation

```bash
ros2 launch gazebo_sim sim.launch.py
```

Menampilkan:

* World indoor 8x8 meter
* Obstacle (sofa, meja, kursi, dll)
* Robot omni 4WD

---

## 🗺️ 2. Jalankan SLAM Toolbox

```bash
ros2 launch slam_toolbox online_async_launch.py \
params_file:=src/gazebo_sim/config/slam.yaml
```

Fungsi:

* Membuat peta real-time
* Publish `/map`

---

## 🧭 3. Jalankan A* Planner

```bash
ros2 run robot_controller astar_planner
```

Fungsi:

* Menerima goal dari RViz2
* Menghasilkan path (`/path`)

---

## 🤖 4. Jalankan Path Follower

```bash
ros2 run robot_controller path_follower
```

Fungsi:

* Pure Pursuit tracking
* Obstacle avoidance (LiDAR)
* Control `/cmd_vel`

---

## 👁️ 5. Jalankan RViz2

```bash
ros2 launch robot_controller rviz.launch.py
```

Gunakan RViz untuk:

* 2D Pose Estimate
* 2D Nav Goal
* Visualisasi path & map

---

# 🎮 Cara Menggunakan Sistem

1. Jalankan semua node (Gazebo + SLAM + Planner + Follower + RViz)
2. Di RViz:

   * Set **2D Pose Estimate** (posisi awal robot)
   * Klik **2D Nav Goal** (target tujuan)
3. Sistem akan otomatis:

   * Membuat map
   * Menghitung path (A*)
   * Mengikuti path (Pure Pursuit)
   * Menghindari obstacle
   * Stop saat mencapai goal

---

# ⚙️ Parameter Penting (Tuning)

## 🚗 Motion Control

```python
LOOKAHEAD_DIST = 0.7
GOAL_REACH_DIST = 0.25
CMD_SMOOTHING = 0.2
```

## 🛑 Safety System

```python
STOP_DISTANCE = 0.25
SLOW_DISTANCE = 0.5
CRITICAL_DISTANCE = 0.15
```

## ⚡ Velocity Limit

```python
MAX_LINEAR_VEL = 0.4
MAX_LATERAL_VEL = 0.35
MAX_ANGULAR_VEL = 0.8
```

---

# 🧯 Troubleshooting

## ❌ Robot tidak bergerak

* Pastikan `/cmd_vel` aktif
* Cek `/path` dari A* planner

## ❌ Gazebo tidak muncul

```bash
killall gzserver gzclient
```

## ❌ Build error ROS2

```bash
rm -rf build install log
colcon build --symlink-install
```

## ❌ SLAM tidak jalan

* Pastikan LiDAR topic `/scan` aktif

---

# 📌 Notes & Best Practices

* Selalu source workspace sebelum run:

```bash
source ~/astar_ws/install/setup.bash
```

* Jalankan node di terminal terpisah untuk debugging
* Gunakan Gazebo Classic dengan performa stabil (RTF > 0.9)
* Jangan jalankan semua node dalam satu terminal

---

# 🔬 Keunggulan Sistem Ini

* Tidak menggunakan Nav2 (lebih ringan & modular)
* Mudah dimodifikasi untuk penelitian
* Cocok untuk:

  * Skripsi
  * Riset robotika
  * Simulasi autonomous navigation

---

# 👨‍💻 Author

Nexus Omni Robotics System
ROS2 Humble + Gazebo Classic Implementation
