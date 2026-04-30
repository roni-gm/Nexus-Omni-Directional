# 🤖 Nexus Omni 4WD — Navigasi Otonom dengan Algoritma A* di ROS2 Humble + Gazebo Classic

> **Studi Kasus:** Implementasi *Motion Planning* berbasis Algoritma A* (A-Star) pada Robot Omni-Wheel 4 Roda (Nexus Omni 4WD) dalam Lingkungan Indoor Berhalangan menggunakan ROS2 Humble, Gazebo Classic, SLAM Toolbox, dan Pure Pursuit Controller.

---

## 📋 Daftar Isi

1. [Deskripsi Proyek](#-deskripsi-proyek)
2. [Lingkungan Simulasi](#-lingkungan-simulasi)
3. [Alasan Menggunakan Algoritma A*](#-alasan-menggunakan-algoritma-a)
4. [Arsitektur Sistem](#-arsitektur-sistem)
5. [Struktur Workspace](#-struktur-workspace)
6. [Deskripsi Package](#-deskripsi-package)
7. [Daftar ROS2 Node](#-daftar-ros2-node)
8. [Daftar ROS2 Topic](#-daftar-ros2-topic)
9. [Alur Sistem (System Workflow)](#-alur-sistem-system-workflow)
10. [Cara Instalasi & Build](#-cara-instalasi--build)
11. [Cara Menjalankan Program](#-cara-menjalankan-program)
12. [Cara Penggunaan](#-cara-penggunaan)
13. [Penjelasan Algoritma A*](#-penjelasan-algoritma-a)
14. [Parameter Sistem](#-parameter-sistem)
15. [Troubleshooting](#-troubleshooting)
16. [Kesimpulan & Pengembangan](#-kesimpulan--pengembangan)

---

## 📌 Deskripsi Proyek

Proyek ini mengimplementasikan sistem **navigasi otonom lengkap** pada robot **Nexus Omni 4WD** dalam lingkungan simulasi Gazebo Classic. Robot mampu:

- **Memetakan lingkungan** secara real-time menggunakan SLAM (*Simultaneous Localization and Mapping*)
- **Merencanakan jalur optimal** dari posisi robot ke titik tujuan menggunakan algoritma **A\* (A-Star)**
- **Menghindari obstacle** secara dinamis menggunakan data LiDAR
- **Melakukan replanning otomatis** saat terdeteksi obstacle baru yang menghalangi jalur
- **Mengikuti jalur** secara presisi menggunakan controller **Pure Pursuit** dengan PID

### Tujuan Utama

| Tujuan | Keterangan |
|--------|-----------|
| Path Planning | Robot menemukan jalur terpendek dan aman dari Start ke Goal |
| Obstacle Avoidance | Robot berhenti dan replan saat ada obstacle di jalur |
| Navigasi Omni | Memanfaatkan kemampuan gerak lateral robot omni-wheel |
| Simulasi Realistis | Environment indoor dengan berbagai obstacle (meja, kursi, sofa, dll) |

---

## 🗺️ Lingkungan Simulasi

### World File

File world yang digunakan: `indoor_obstacles.world`

Robot beroperasi di dalam **ruangan indoor 8m × 8m** yang mensimulasikan kondisi nyata seperti ruang kantor atau ruang tamu.

### Denah Lingkungan

```
+--------------------------------------------------+  Y = +4m
|                                                  |
|  [SOFA 1.8x0.8]   [BOOKSHELF]                   |
|                                                  |
|  [COFFEE TABLE]              [COFFEE TABLE]      |
|                  [PILLAR]                        |
|      [DINING TABLE]                              |
|      [CHAIR N] [CHAIR S]                         |
| [PARTITION_L] [PARTITION_R]                      |
|                   [TV_STAND]                     |
| ★ START (-3,-3)                                  |
+--------------------------------------------------+  Y = -4m
X = -4m                                   X = +4m
```

### Daftar Obstacle

| Obstacle | Dimensi (m) | Posisi (x, y) | Keterangan |
|----------|-------------|---------------|-----------|
| Dinding Utara | 8.15 × 0.15 × 0.5 | (0, 4.075) | Batas ruangan |
| Dinding Selatan | 8.15 × 0.15 × 0.5 | (0, -4.075) | Batas ruangan |
| Dinding Timur | 0.15 × 8.0 × 0.5 | (4.075, 0) | Batas ruangan |
| Dinding Barat | 0.15 × 8.0 × 0.5 | (-4.075, 0) | Batas ruangan |
| Sofa | 1.8 × 0.8 × 0.45 | (-2.5, 2.5) | Obstacle besar |
| Coffee Table | 0.6 × 0.6 × 0.4 | (-2.5, 1.3) | Obstacle sedang |
| Dining Table | 1.2 × 0.8 × 0.75 | (1.0, 0.0) | Obstacle besar |
| Chair North | 0.45 × 0.45 × 0.45 | (1.0, 0.75) | Obstacle kecil |
| Chair South | 0.45 × 0.45 × 0.45 | (1.0, -0.75) | Obstacle kecil |
| Bookshelf | 0.4 × 1.5 × 1.8 | (3.35, 3.0) | Obstacle tinggi |
| TV Stand | 1.4 × 0.4 × 0.4 | (0.0, -3.3) | Obstacle sedang |
| Pillar | r=0.12, h=1.0 | (-0.5, -1.5) | Tiang silindris |
| Partition Left | 0.1 × 1.2 × 0.5 | (-1.5, -2.5) | Koridor sempit |
| Partition Right | 0.1 × 1.2 × 0.5 | (-0.7, -2.5) | Koridor sempit |

### Posisi Start & Goal

| | Koordinat (x, y, z) | Keterangan |
|--|---------------------|-----------|
| **Start** | (-3.0, -3.0, 0.071) | Sudut barat daya ruangan (area bebas) |
| **Goal** | Ditentukan pengguna via RViz2 | Klik **2D Goal Pose** di RViz2 |

> **Cara menetapkan Goal:** Gunakan tombol `2D Goal Pose` di toolbar RViz2, klik pada posisi yang diinginkan di map. A* akan otomatis merencanakan jalur dari posisi robot saat ini ke titik tersebut.

---

## ⭐ Alasan Menggunakan Algoritma A*

### Perbandingan dengan Algoritma Lain

| Algoritma | Optimal? | Lengkap? | Kecepatan | Cocok untuk Indoor? |
|-----------|----------|----------|-----------|---------------------|
| **A\*** | ✅ Ya | ✅ Ya | ⚡ Cepat | ✅ **Sangat Cocok** |
| Dijkstra | ✅ Ya | ✅ Ya | 🐢 Lambat | ⚠️ Kurang efisien |
| BFS | ❌ Tidak | ✅ Ya | 🐢 Lambat | ❌ Tidak optimal |
| DFS | ❌ Tidak | ❌ Tidak | ⚡ Cepat | ❌ Tidak aman |
| RRT | ❌ Tidak | ✅ Ya | ⚡ Cepat | ⚠️ Path kasar |
| Potential Field | ❌ Tidak | ❌ Tidak | ⚡ Cepat | ❌ Local minima |

### Keunggulan A* untuk Proyek Ini

1. **Optimalitas terjamin** — A* menghasilkan jalur terpendek selama heuristic admissible (tidak overestimate). Pada proyek ini digunakan *Octile Distance* yang optimal untuk gerakan 8-arah.

2. **Efisiensi tinggi** — A* tidak mengeksplorasi seluruh grid seperti Dijkstra. Heuristic mengarahkan pencarian ke arah goal sehingga jauh lebih cepat.

3. **Kompatibel dengan Occupancy Grid** — Map hasil SLAM disimpan sebagai occupancy grid, yang merupakan input alami untuk A*. Setiap sel grid langsung menjadi node dalam graph pencarian.

4. **Mendukung Obstacle Inflation** — A* dapat mempertimbangkan *inflation radius* robot sehingga jalur yang dihasilkan selalu aman dari obstacle, sesuai dimensi fisik robot.

5. **Mendukung Replanning Dinamis** — A* dapat dipanggil ulang kapan saja saat kondisi berubah (obstacle baru terdeteksi), menghasilkan jalur baru yang optimal tanpa overhead besar.

6. **Cost function fleksibel** — Pada implementasi ini, A* menggunakan *obstacle proximity cost* tambahan sehingga jalur yang dihasilkan lebih memilih melewati area yang jauh dari obstacle.

---

## 🏗️ Arsitektur Sistem

```
                    ┌─────────────────────────────────────┐
                    │           GAZEBO CLASSIC             │
                    │  ┌─────────┐    ┌───────────────┐   │
                    │  │  World  │    │  Nexus Omni   │   │
                    │  │(indoor_ │    │   4WD Robot   │   │
                    │  │obstacle)│    │   + LiDAR     │   │
                    │  └─────────┘    └───────────────┘   │
                    └──────────┬──────────────┬───────────┘
                               │ /odom        │ /scan
                               ▼              ▼
              ┌────────────────────────────────────────┐
              │           SLAM TOOLBOX                  │
              │  Localization + Mapping                 │
              │  Input:  /scan + /odom + /tf            │
              │  Output: /map + /tf (map→odom)          │
              └───────────────┬────────────────────────┘
                              │ /map
                              ▼
              ┌────────────────────────────────────────┐
              │           A* PLANNER NODE               │
              │  Input:  /map + /odom + /goal_pose      │
              │  Proses: Build Grid → Inflate → A*      │
              │          → Smooth Path                  │
              │  Output: /path                          │
              │  + Dynamic Replanning (0.5s interval)   │
              └───────────────┬────────────────────────┘
                              │ /path
                              ▼
              ┌────────────────────────────────────────┐
              │         PATH FOLLOWER NODE              │
              │  Input:  /path + /odom + /scan          │
              │  Proses: Pure Pursuit + PID             │
              │          + Obstacle Avoidance           │
              │  Output: /cmd_vel                       │
              └───────────────┬────────────────────────┘
                              │ /cmd_vel
                              ▼
              ┌────────────────────────────────────────┐
              │       OMNI KINEMATICS NODE              │
              │  Input:  /cmd_vel                       │
              │  Proses: IK Matrix (vx,vy,ω → wheels)  │
              │  Output: /wheel_velocities              │
              └────────────────────────────────────────┘
```

---

## 📁 Struktur Workspace

```
astar_ws/
├── src/                          # Source packages ROS2
│   ├── astar_planner/            # Package: Path Planning (A*)
│   ├── gazebo_sim/               # Package: Simulasi & Launch Files
│   ├── robot_controller/         # Package: Kontroler Robot
│   └── robot_description/        # Package: Deskripsi Robot (URDF/XACRO)
│
├── build/                        # Hasil build (auto-generated)
├── install/                      # Hasil install (auto-generated)
└── log/                          # Log build & runtime (auto-generated)
```

> **Catatan:** Folder `build/`, `install/`, dan `log/` dibuat otomatis saat proses `colcon build`. Tidak perlu dimodifikasi secara manual.

---

## 📦 Deskripsi Package

### 1. `astar_planner`

**Fungsi:** Merencanakan jalur optimal dari posisi robot ke titik goal menggunakan algoritma A*.

```
astar_planner/
├── astar_planner/
│   ├── __init__.py
│   └── astar_planner.py      ← Node utama A* dengan dynamic replanning
├── package.xml
├── setup.cfg
└── setup.py
```

**Komponen utama dalam `astar_planner.py`:**

| Kelas | Fungsi |
|-------|--------|
| `GridBuilder` | Konversi `OccupancyGrid` → binary grid + obstacle inflation menggunakan `scipy.ndimage.distance_transform_edt` |
| `AStarCore` | Implementasi algoritma A* dengan 8-directional movement, obstacle proximity cost, dan octile heuristic |
| `AStarPlanner` | ROS2 Node: subscribe `/map`, `/odom`, `/goal_pose`, `/scan` → publish `/path` dengan replanning dinamis |

---

### 2. `gazebo_sim`

**Fungsi:** Menyediakan konfigurasi simulasi, world file, map, launch files, dan konfigurasi SLAM.

```
gazebo_sim/
├── config/
│   ├── nav.rviz              ← Konfigurasi tampilan RViz2
│   └── slam.yaml             ← Parameter SLAM Toolbox
├── launch/
│   ├── sim.launch.py         ← Launch Gazebo + spawn robot
│   ├── nav.launch.py         ← Launch SLAM + A* + path follower
│   ├── rviz.launch.py        ← Launch RViz2
│   └── slam.launch.py        ← Launch SLAM saja (standalone)
├── maps/
│   ├── indoor_obstacles.pgm  ← Peta ruangan indoor (image)
│   ├── indoor_obstacles.yaml ← Metadata peta (resolusi, origin)
│   ├── map.pgm               ← Peta alternatif
│   └── map.yaml              ← Metadata peta alternatif
└── worlds/
    ├── indoor_obstacles.world ← World aktif (8x8m dengan obstacle)
    └── nexus.world            ← World alternatif
```

---

### 3. `robot_controller`

**Fungsi:** Mengeksekusi perintah gerak robot berdasarkan path yang diberikan planner.

```
robot_controller/
├── robot_controller/
│   ├── __init__.py
│   ├── path_follower.py      ← Pure Pursuit Controller + PID + Obstacle Avoidance
│   ├── omni_kinematics.py    ← Inverse Kinematics roda omni 4WD
│   └── config/
│       └── controllers.yaml  ← Konfigurasi controller
├── package.xml
├── setup.cfg
└── setup.py
```

**Komponen utama:**

| File | Kelas/Fungsi | Fungsi |
|------|-------------|--------|
| `path_follower.py` | `PIDController` | Controller PID untuk tracking heading dan lateral |
| `path_follower.py` | `PurePursuitController` | Adaptive lookahead pure pursuit untuk path following |
| `path_follower.py` | `PathFollower` (Node) | Node utama: subscribe `/path`, `/odom`, `/scan` → publish `/cmd_vel` |
| `omni_kinematics.py` | `OmniKinematics` (Node) | Konversi `Twist (vx, vy, ω)` → kecepatan roda individual |

---

### 4. `robot_description`

**Fungsi:** Mendefinisikan model fisik robot (geometri, massa, inersia, sensor, plugin Gazebo).

```
robot_description/
├── urdf/
│   ├── robot.urdf.xacro          ← Entry point URDF
│   ├── robot_core_highres.xacro  ← Definisi lengkap robot + plugin
│   └── inertial_macros.xacro     ← Makro inersia
└── meshes/
    ├── base_link.STL             ← Mesh chassis
    ├── lidar.dae                 ← Mesh sensor LiDAR
    ├── wheel1-4_Link.STL         ← Mesh roda
    ├── motor1-4_Link.STL         ← Mesh motor
    └── rubber1-4_Link.STL        ← Mesh rubber wheel
```

**Spesifikasi Robot Nexus Omni 4WD:**

| Parameter | Nilai |
|-----------|-------|
| Chassis | 0.28m × 0.14m |
| Wheel Radius | 0.033m |
| Robot Bounding Radius | ~0.18m |
| Total Mass | ~7.30 kg |
| Sensor | RPLiDAR (360°, range 0.12–8.0m, 10Hz) |
| Plugin Gazebo | `libgazebo_ros_planar_move.so` |
| Command Topic | `/cmd_vel` |
| Odometry Topic | `/odom` |

---

## 🔵 Daftar ROS2 Node

| Node | Package | Fungsi |
|------|---------|--------|
| `/astar_planner` | `astar_planner` | Menerima map SLAM dan goal pose, menjalankan algoritma A* untuk menghasilkan jalur optimal, melakukan replanning dinamis setiap 0.5 detik |
| `/path_follower` | `robot_controller` | Mengikuti jalur dari A* menggunakan Pure Pursuit + PID, mendeteksi obstacle via LiDAR, publish perintah kecepatan ke `/cmd_vel` |
| `/omni_kinematics` | `robot_controller` | Mengkonversi `Twist (vx, vy, ω)` menjadi kecepatan angular setiap roda menggunakan inverse kinematics matrix |
| `/slam_toolbox` | `slam_toolbox` | Membangun peta lingkungan secara real-time dari data LiDAR dan odometry, menerbitkan `/map` dan transformasi `map → odom` |
| `/robot_state_publisher` | `robot_state_publisher` | Membaca URDF robot dan menerbitkan transformasi TF statis antar link robot (chassis, roda, sensor) |
| `/gazebo` | `gazebo_ros` | Server simulasi fisika Gazebo, menjalankan simulasi dunia virtual termasuk robot, obstacle, dan sensor |
| `/gazebo_ros_planar_move` | `gazebo_ros` | Plugin Gazebo yang menerima perintah `/cmd_vel` dan menerapkan gerak pada robot di simulasi, serta menerbitkan odometry |
| `/laser_controller` | `gazebo_ros` | Plugin Gazebo yang mensimulasikan sensor LiDAR dan menerbitkan data scan ke topic `/scan` |
| `/rviz2` | `rviz2` | Visualisasi real-time: peta, jalur robot, data LiDAR, model robot 3D, dan pose robot |
| `/transform_listener_impl_xxx` | Internal | Node internal ROS2 untuk memproses TF tree, dibuat otomatis oleh node yang menggunakan transformasi |

---

## 🟢 Daftar ROS2 Topic

| Topic | Tipe Pesan | Publisher | Subscriber | Fungsi |
|-------|-----------|-----------|-----------|--------|
| `/scan` | `sensor_msgs/LaserScan` | `/laser_controller` | `/slam_toolbox`, `/astar_planner`, `/path_follower` | Data mentah LiDAR 360°: jarak obstacle per sudut |
| `/odom` | `nav_msgs/Odometry` | `/gazebo_ros_planar_move` | `/slam_toolbox`, `/astar_planner`, `/path_follower` | Pose dan kecepatan robot dari odometri roda |
| `/map` | `nav_msgs/OccupancyGrid` | `/slam_toolbox` | `/astar_planner` | Peta occupancy grid hasil SLAM (0=bebas, 100=obstacle, -1=unknown) |
| `/goal_pose` | `geometry_msgs/PoseStamped` | `RViz2 (user)` atau `/path_follower` | `/astar_planner` | Titik tujuan navigasi, dikirim saat user klik 2D Goal Pose di RViz2 |
| `/path` | `nav_msgs/Path` | `/astar_planner` | `/path_follower` | Jalur optimal hasil A*: urutan waypoint dalam frame `map` |
| `/cmd_vel` | `geometry_msgs/Twist` | `/path_follower` | `/gazebo_ros_planar_move`, `/omni_kinematics` | Perintah kecepatan robot: `linear.x`, `linear.y`, `angular.z` |
| `/wheel_velocities` | `std_msgs/Float64MultiArray` | `/omni_kinematics` | (monitoring) | Kecepatan angular setiap roda [W1, W2, W3, W4] dalam rad/s |
| `/tf` | `tf2_msgs/TFMessage` | `/slam_toolbox`, `/robot_state_publisher`, `/gazebo_ros_planar_move` | Semua node | Transformasi dinamis antar frame: `map→odom`, `odom→base_link` |
| `/tf_static` | `tf2_msgs/TFMessage` | `/robot_state_publisher` | Semua node | Transformasi statis antar link robot: `base_link→chassis`, `chassis→laser` |
| `/robot_description` | `std_msgs/String` | `/robot_state_publisher` | `spawn_entity.py` | String URDF robot untuk spawn di Gazebo |

### Hubungan Antar Topic

```
/scan ──────────────────► /slam_toolbox ──► /map ──► /astar_planner ──► /path
         │                                                    ▲               │
         │                                                    │               ▼
         └──────────────────────────────────────► /path_follower ──► /cmd_vel
/odom ─────────────────────────────────────────► /path_follower        │
/odom ──────────────────────────────────────────────────────────────────┘
/tf ──────────────────────────────────────────────────────────────────────►
/goal_pose ──────────────────────────────────────► /astar_planner
```

---

## 🔄 Alur Sistem (System Workflow)

### Fase 1: Inisialisasi

```
1. Gazebo Classic start → World indoor_obstacles.world di-load
2. Robot Nexus Omni spawn di posisi (-3.0, -3.0, 0.071)
3. robot_state_publisher parse URDF → publish TF statis
4. plugin planar_move aktif → mulai publish /odom
5. LiDAR aktif → mulai publish /scan
```

### Fase 2: SLAM (Pemetaan)

```
/scan + /odom + /tf
        │
        ▼
SLAM Toolbox (sync mode)
        │
        ├─► /map (OccupancyGrid, update setiap 2 detik)
        └─► /tf  (transformasi map → odom, update 20Hz)
```

> SLAM Toolbox menggunakan algoritma *Karto SLAM* untuk membangun peta secara bertahap. Setiap update scan LiDAR dicocokkan dengan peta yang sudah ada (*scan matching*) untuk menentukan posisi robot yang lebih akurat.

### Fase 3: Path Planning (A*)

```
/map + /odom + /goal_pose
        │
        ▼
┌─────────────────────────────────────────┐
│  1. Build OccupancyGrid → Binary Grid   │
│  2. Distance Transform (scipy EDT)      │
│  3. Inflate obstacles by 0.25m          │
│  4. Convert world pose → grid cell      │
│  5. Run A* (8-directional, octile h)    │
│     with obstacle proximity cost        │
│  6. Smooth path (moving average)        │
│  7. Publish /path                       │
│  8. Timer 0.5s → check replan needed   │
└─────────────────────────────────────────┘
        │
        ▼
/path (nav_msgs/Path)
```

### Fase 4: Path Following

```
/path + /odom + /scan
        │
        ▼
┌─────────────────────────────────────────┐
│  1. Prune completed waypoints           │
│  2. Adaptive Pure Pursuit lookahead     │
│     (0.25m - 0.50m, based on speed)    │
│  3. PID angular + lateral control      │
│  4. Obstacle detection ±60° from front │
│     - dist < 0.10m → Emergency STOP    │
│     - dist < 0.25m → Slow + lateral    │
│  5. Apply velocity & accel limits      │
│  6. Smoothing (moving average 5 frame) │
│  7. Publish /cmd_vel                    │
└─────────────────────────────────────────┘
        │
        ▼
/cmd_vel (geometry_msgs/Twist)
        │
        ▼
planar_move plugin → Robot bergerak di Gazebo
```

### Fase 5: Dynamic Replanning

```
Setiap 0.5 detik, A* Node cek:
  ┌─ Apakah robot menyimpang > 15cm dari path? ─► REPLAN
  ├─ Apakah obstacle baru menghalangi path?    ─► REPLAN
  └─ Apakah goal terblokir obstacle?           ─► REPLAN
                    │
                    ▼
           Plan ulang dengan map terbaru
           Robot berhenti sementara
           Path baru di-publish ke /path
```

---

## 💻 Cara Instalasi & Build

### Prasyarat

```bash
# ROS2 Humble
sudo apt install ros-humble-desktop

# Dependencies tambahan
sudo apt install \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-slam-toolbox \
  ros-humble-robot-state-publisher \
  ros-humble-xacro \
  python3-scipy

# Source ROS2
source /opt/ros/humble/setup.bash
```

### Build Workspace

```bash
# Clone / extract workspace
cd ~
# (ekstrak astar_ws ke home directory)

# Build semua package
cd ~/astar_ws
colcon build --symlink-install

# Source workspace
source install/setup.bash

# (Opsional) Tambahkan ke .bashrc agar otomatis
echo "source ~/astar_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Verifikasi Build

```bash
# Cek semua package berhasil di-build
ros2 pkg list | grep -E "astar_planner|gazebo_sim|robot_controller|robot_description"

# Expected output:
# astar_planner
# gazebo_sim
# robot_controller
# robot_description
```

---

## 🚀 Cara Menjalankan Program

Program dijalankan dalam **3 terminal terpisah** secara berurutan:

### Terminal 1 — Simulasi Gazebo

```bash
source ~/astar_ws/install/setup.bash
ros2 launch gazebo_sim sim.launch.py
```

**Fungsi `sim.launch.py`:**
- Menjalankan `gzserver` (physics engine Gazebo) dengan world `indoor_obstacles.world`
- Menjalankan `robot_state_publisher` untuk publish TF dari URDF
- Spawn robot Nexus Omni di posisi `(-3.0, -3.0, 0.071)` setelah 5 detik
- Menjalankan `gzclient` (Gazebo GUI) setelah 10 detik

> **Mode Headless** (tanpa GUI Gazebo, lebih ringan):
> ```bash
> ros2 launch gazebo_sim sim.launch.py headless:=true
> ```

**Tunggu** hingga muncul pesan:
```
[spawn_entity.py] SpawnEntity: Successfully spawned entity [nexus_robot]
[gzserver] [INFO] Connected to Gazebo master
```

---

### Terminal 2 — Navigation Stack

```bash
source ~/astar_ws/install/setup.bash
ros2 launch gazebo_sim nav.launch.py
```

**Fungsi `nav.launch.py`:**
- Menjalankan **SLAM Toolbox** (`sync_slam_toolbox_node`) dengan parameter dari `slam.yaml`
  - Membangun peta dari data `/scan`
  - Menerbitkan `/map` dan TF `map → odom`
- Menjalankan **A\* Planner** setelah 3 detik (menunggu SLAM siap)
  - Subscribe `/map`, `/odom`, `/goal_pose`, `/scan`
  - Publish `/path`
  - Melakukan replanning dinamis setiap 0.5 detik
- Menjalankan **Path Follower** (langsung)
  - Subscribe `/path`, `/odom`, `/scan`
  - Publish `/cmd_vel`

**Tunggu** hingga muncul pesan:
```
[astar_planner] AStar Planner Ready - WITH DYNAMIC REPLANNING
[path_follower] PATH FOLLOWER ULTRA PRECISE VERSION
```

---

### Terminal 3 — Visualisasi RViz2

```bash
source ~/astar_ws/install/setup.bash
ros2 launch gazebo_sim rviz.launch.py
```

**Fungsi `rviz.launch.py`:**
- Menjalankan `robot_state_publisher` untuk TF
- Membuka RViz2 dengan konfigurasi `nav.rviz`
- Menampilkan: peta SLAM, jalur robot, data LiDAR, model robot 3D

**Setup tampilan RViz2 (jika tidak otomatis):**
1. Tambahkan display **Map** → Topic: `/map`
2. Tambahkan display **Path** → Topic: `/path`
3. Tambahkan display **LaserScan** → Topic: `/scan`
4. Tambahkan display **RobotModel**
5. Set **Fixed Frame** → `map`

---

### (Opsional) Terminal 4 — SLAM Standalone

```bash
source ~/astar_ws/install/setup.bash
ros2 launch gazebo_sim slam.launch.py
```

Digunakan untuk menjalankan SLAM saja tanpa stack navigasi penuh. Berguna saat ingin melakukan pemetaan manual terlebih dahulu sebelum navigasi otonom.

---

## 🎮 Cara Penggunaan

### Langkah 1: Verifikasi Sistem Berjalan

```bash
# Cek node yang berjalan
ros2 node list
# Expected: /astar_planner, /path_follower, /slam_toolbox, /robot_state_publisher, dll

# Cek topic aktif
ros2 topic list | grep -E "map|path|cmd_vel|scan|odom"

# Cek data LiDAR
ros2 topic hz /scan
# Expected: ~10 Hz

# Cek odometry
ros2 topic echo /odom --once
# Expected: pose dan twist robot
```

### Langkah 2: Verifikasi SLAM Berjalan

Buka RViz2 dan pastikan:
- **Peta** mulai terbentuk (area putih = bebas, abu-abu = obstacle)
- **Robot marker** terlihat di posisi (-3, -3)
- **LiDAR scan** terlihat sebagai titik-titik merah di sekitar robot

### Langkah 3: Set Goal dengan 2D Goal Pose

1. Di toolbar RViz2, klik tombol **`2D Goal Pose`** (ikon panah hijau)
2. **Klik dan drag** pada posisi yang diinginkan di peta
   - Klik menentukan posisi (x, y)
   - Arah drag menentukan orientasi (heading) goal
3. A* Planner akan langsung menerima goal dan merencanakan jalur
4. Jalur akan ditampilkan sebagai **garis hijau** di RViz2
5. Robot akan **mulai bergerak** mengikuti jalur

### Apa yang Terjadi Setelah Goal Diberikan

```
User klik 2D Goal Pose di RViz2
        │
        ▼
RViz2 publish → /goal_pose (geometry_msgs/PoseStamped)
        │
        ▼
A* Planner terima goal
→ Build grid dari /map terbaru
→ Inflate obstacle 0.25m
→ Run A* dari posisi robot ke goal
→ Smooth path
→ Publish /path
        │
        ▼
Path Follower terima /path
→ Adaptive Pure Pursuit
→ Tracking waypoint demi waypoint
→ Slow down / stop jika obstacle di depan
→ Trigger replan jika obstacle > 1.2 detik
        │
        ▼
Robot bergerak → mencapai Goal
→ "✅ GOAL REACHED!" tercetak di terminal
→ Robot BERHENTI dan mengunci posisi
```

### Cara Set Goal via Command Line (Tanpa RViz2)

```bash
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped \
  "{ header: {frame_id: 'map'},
     pose: {
       position: {x: 2.0, y: 1.0, z: 0.0},
       orientation: {w: 1.0}
     }
   }" --once
```

---

## 🧮 Penjelasan Algoritma A*

### Konsep Dasar

A* adalah algoritma pencarian jalur (*path finding*) yang menemukan jalur terpendek dari titik **start** ke titik **goal** dalam sebuah graph, dengan menggunakan fungsi evaluasi:

```
f(n) = g(n) + h(n)
```

| Simbol | Nama | Arti |
|--------|------|------|
| `f(n)` | Total cost | Total estimasi biaya jalur melalui node `n` |
| `g(n)` | Actual cost | Biaya aktual dari start ke node `n` |
| `h(n)` | Heuristic | Estimasi biaya dari node `n` ke goal |

### Implementasi dalam Proyek Ini

#### 1. Representasi Map

```python
# OccupancyGrid dari SLAM dikonversi ke binary grid
# dengan obstacle inflation menggunakan Distance Transform

binary = (data >= OBSTACLE_THRESHOLD)  # threshold = 65

# Distance Transform: hitung jarak setiap cell ke obstacle terdekat
dist = distance_transform_edt(~binary) * resolution

# Cell dianggap terblokir jika jarak ke obstacle < inflation_radius
occupied = dist < INFLATION_RADIUS  # 0.25m
```

#### 2. Heuristic — Octile Distance

```python
def heuristic(self, a, b):
    dx = abs(a[0] - b[0])
    dy = abs(a[1] - b[1])
    # Octile: optimal untuk 8-directional movement
    return max(dx, dy) + (sqrt(2) - 1) * min(dx, dy)
```

Octile distance dipilih karena robot bergerak dalam 8 arah (4 cardinal + 4 diagonal), sehingga heuristic ini tidak *overestimate* (admissible) dan A* tetap menghasilkan jalur optimal.

#### 3. Obstacle Proximity Cost

```python
# Tambahan cost untuk area dekat obstacle
# Mendorong A* memilih jalur yang lebih jauh dari obstacle
if dist < SAFE_DISTANCE:  # 0.28m
    d = max(dist, ROBOT_RADIUS)
    obstacle_cost = OBSTACLE_COST_GAIN * ((SAFE_DISTANCE - d) / SAFE_DISTANCE) ** 2

new_cost = g_score[curr] + base_cost + obstacle_cost
```

#### 4. Gerakan 8-Arah

```python
DIRS = [
    (1, 0, 1.0),    # Kanan
    (-1, 0, 1.0),   # Kiri
    (0, 1, 1.0),    # Atas
    (0, -1, 1.0),   # Bawah
    (1, 1, 1.414),  # Kanan-Atas (diagonal)
    (1, -1, 1.414), # Kanan-Bawah
    (-1, 1, 1.414), # Kiri-Atas
    (-1, -1, 1.414),# Kiri-Bawah
]
```

#### 5. Dynamic Replanning

```python
# Setiap 0.5 detik, cek kondisi replanning
def check_and_replan(self):
    if self.replan_required:          # Obstacle baru di path
        should_replan = True
    if self.check_deviation_from_path():  # Robot menyimpang > 15cm
        should_replan = True
    if self.is_goal_blocked():            # Goal terblokir
        should_replan = True

    if should_replan:
        self.stop_robot()
        self.plan_and_publish()  # Jalankan A* ulang
```

### Visualisasi Cara Kerja A*

```
Map Grid (S=start, G=goal, X=obstacle, .=bebas):

. . . . . X X . G
. . . . . X X . .
S . . . . . . . .
. . . . . . . . .

Eksplorasi A* (angka = urutan eksplorasi):
1 3 5 7 . X X . G
2 4 6 8 . X X . .
S 1 2 3 4 5 6 7 8
. . . . . . . . .

Jalur terpendek yang ditemukan (→):
. . . . . X X → G
. . . . . X X ↑ .
S → → → → → ↑ . .
. . . . . . . . .
```

---

## ⚙️ Parameter Sistem

### A* Planner (`astar_planner.py`)

| Parameter | Nilai | Keterangan |
|-----------|-------|-----------|
| `ROBOT_RADIUS` | 0.18 m | Radius bounding circle robot |
| `SAFETY_MARGIN` | 0.07 m | Clearance tambahan di atas robot radius |
| `INFLATION_RADIUS` | 0.25 m | Total radius inflation obstacle |
| `MAP_RESOLUTION` | 0.05 m/cell | Resolusi grid (5cm per cell) |
| `OBSTACLE_THRESHOLD` | 65 | Nilai occupancy dianggap obstacle (0-100) |
| `SAFE_DISTANCE` | 0.28 m | Jarak aman dari obstacle untuk cost extra |
| `OBSTACLE_COST_GAIN` | 5.0 | Bobot penalty area dekat obstacle |
| `REPLAN_INTERVAL` | 0.5 s | Interval cek replanning |
| `REPLAN_DISTANCE_THRESHOLD` | 0.15 m | Deviasi robot dari path sebelum replan |

### Path Follower (`path_follower.py`)

| Parameter | Nilai | Keterangan |
|-----------|-------|-----------|
| `LOOKAHEAD_DIST` | 0.35 m | Jarak lookahead default |
| `LOOKAHEAD_MIN` | 0.25 m | Jarak lookahead minimum (saat belok) |
| `LOOKAHEAD_MAX` | 0.50 m | Jarak lookahead maksimum (saat lurus) |
| `MAX_LINEAR_VEL` | 0.40 m/s | Kecepatan linear maksimum |
| `MAX_LATERAL_VEL` | 0.35 m/s | Kecepatan lateral maksimum (omni) |
| `MAX_ANGULAR_VEL` | 1.2 rad/s | Kecepatan angular maksimum |
| `GOAL_REACH_DIST` | 0.08 m | Jarak untuk goal dianggap tercapai |
| `STOP_DISTANCE` | 0.10 m | Jarak emergency stop dari obstacle |
| `SLOW_DISTANCE` | 0.25 m | Jarak mulai mengurangi kecepatan |
| `PID_KP` | 1.2 | Gain proporsional PID |
| `PID_KI` | 0.05 | Gain integral PID |
| `PID_KD` | 0.8 | Gain derivatif PID |

---

## 🔧 Troubleshooting

### Masalah & Solusi

#### Robot tidak bergerak setelah goal diberikan

```bash
# 1. Cek apakah A* menerima goal
ros2 topic echo /goal_pose --once

# 2. Cek apakah path ter-publish
ros2 topic echo /path --once | head -20

# 3. Cek apakah cmd_vel ter-publish
ros2 topic echo /cmd_vel

# 4. Cek apakah planar_move subscribe cmd_vel
ros2 topic info /cmd_vel
# Expected: Subscription count: 1 (dari planar_move plugin)
```

#### SLAM tidak membangun peta

```bash
# Cek TF chain
ros2 run tf2_tools view_frames
# Pastikan: map → odom → base_link → laser tersambung

# Cek scan topic
ros2 topic hz /scan
# Expected: ~10 Hz
```

#### A* gagal menemukan path

```bash
# Cek log A* planner
ros2 topic echo /rosout | grep astar

# Kemungkinan penyebab:
# 1. Inflation radius terlalu besar → kurangi SAFETY_MARGIN
# 2. Goal di dalam obstacle → pindah goal ke area bebas
# 3. Map belum penuh terpetakan → gerakkan robot manual dulu
```

#### Gazebo freeze / crash

```bash
# Jalankan headless
ros2 launch gazebo_sim sim.launch.py headless:=true

# Atau force software rendering
LIBGL_ALWAYS_SOFTWARE=1 ros2 launch gazebo_sim sim.launch.py
```

#### Robot oscillasi di goal

```bash
# Naikkan GOAL_REACH_DIST di path_follower.py
GOAL_REACH_DIST = 0.15  # dari 0.08 ke 0.15

# Rebuild
colcon build --packages-select robot_controller
source install/setup.bash
```

---

## ✅ Kesimpulan & Pengembangan

### Ringkasan Manfaat Sistem

| Aspek | Pencapaian |
|-------|-----------|
| **Path Planning** | A* menghasilkan jalur terpendek dan aman dengan obstacle inflation |
| **Dynamic Replanning** | Robot dapat replan otomatis saat obstacle baru terdeteksi via LiDAR |
| **Presisi Navigasi** | Pure Pursuit + PID controller dengan toleransi goal 8cm |
| **Keamanan** | Emergency stop saat obstacle < 10cm, slow zone < 25cm |
| **Omni Advantage** | Gerak lateral memungkinkan manuver di ruang sempit tanpa rotate |
| **Pemetaan** | SLAM Toolbox membangun peta real-time dengan resolusi 5cm |

### Potensi Pengembangan

1. **Integrasi Nav2** — Mengganti custom planner/controller dengan Nav2 stack untuk fitur yang lebih lengkap (recovery behaviors, costmap layers, dll)

2. **Multi-Robot Coordination** — Koordinasi beberapa robot Nexus Omni dalam satu environment menggunakan ROS2 multi-robot framework

3. **Deep Learning Obstacle Detection** — Menambahkan kamera + YOLO untuk deteksi obstacle semantik (membedakan manusia, furnitur, dll)

4. **Real Robot Deployment** — Migrasi dari simulasi Gazebo ke robot fisik dengan driver ROS2 yang sesuai

5. **Coverage Path Planning** — Implementasi jalur coverage (boustrophedon / spiral) untuk aplikasi robot vacuum yang menyapu seluruh area

6. **AMCL Localization** — Menambahkan AMCL (*Adaptive Monte Carlo Localization*) untuk estimasi pose yang lebih akurat pada peta yang sudah jadi

7. **Optimasi A* dengan JPS** — Mengganti A* standar dengan *Jump Point Search* untuk pencarian jalur yang lebih cepat pada grid besar

---

## 👤 Informasi Proyek

| | |
|--|--|
| **Maintainer** | ronnigm |
| **Email** | ronignwn2505@gmail.com |
| **ROS Version** | ROS2 Humble Hawksbill |
| **Gazebo Version** | Gazebo Classic 11 |
| **Ubuntu Version** | Ubuntu 22.04 LTS |
| **Python Version** | Python 3.10 |
| **Robot** | Nexus Omni 4WD |

---

*Dokumentasi ini dibuat untuk keperluan akademik dan pengembangan sistem navigasi robot otonom.*
