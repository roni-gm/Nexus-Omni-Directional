# astar_planner.py - WITH DYNAMIC REPLANNING
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from rclpy.duration import Duration

from nav_msgs.msg import Path, OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan

import math
import heapq
import numpy as np
from threading import Lock

# ==========================
# PARAMETER
# ==========================
ROBOT_RADIUS     = 0.18
SAFETY_MARGIN    = 0.07
INFLATION_RADIUS = 0.25

MAP_RESOLUTION     = 0.05
OBSTACLE_THRESHOLD = 65

SAFE_DISTANCE        = 0.28
OBSTACLE_COST_GAIN   = 5.0

# Dynamic replanning parameters
REPLAN_INTERVAL      = 0.5      # detik, cek ulang setiap 500ms
REPLAN_DISTANCE_THRESHOLD = 0.15  # meter, replan jika deviasi > 15cm
OBSTACLE_CHANGE_THRESHOLD = 0.10   # meter, replan jika obstacle lebih dekat

class GridBuilder:
    def __init__(self, inflation_radius, resolution):
        self.inflation_radius = inflation_radius
        self.resolution = resolution

    def build(self, msg: OccupancyGrid):
        w = msg.info.width
        h = msg.info.height
        data = np.array(msg.data, dtype=np.int8).reshape((h, w))
        binary = (data >= OBSTACLE_THRESHOLD)

        from scipy.ndimage import distance_transform_edt
        dist = distance_transform_edt(~binary) * self.resolution

        occupied = dist < self.inflation_radius
        return occupied, dist

    def world_to_grid(self, wx, wy, origin):
        col = int((wx - origin.position.x) / self.resolution)
        row = int((wy - origin.position.y) / self.resolution)
        return col, row

    def grid_to_world(self, col, row, origin):
        wx = col * self.resolution + origin.position.x + self.resolution / 2
        wy = row * self.resolution + origin.position.y + self.resolution / 2
        return wx, wy

class AStarCore:
    DIRS = [
        (1, 0, 1.0), (-1, 0, 1.0),
        (0, 1, 1.0), (0, -1, 1.0),
        (1, 1, 1.414), (1, -1, 1.414),
        (-1, 1, 1.414), (-1, -1, 1.414),
    ]

    def plan(self, occ_grid, dist_grid, start, goal):
        h, w = occ_grid.shape

        if occ_grid[start[1], start[0]]:
            start = self.find_nearest_free(occ_grid, start)
            if start is None:
                return None

        if occ_grid[goal[1], goal[0]]:
            goal = self.find_nearest_free(occ_grid, goal)
            if goal is None:
                return None

        open_set = []
        counter = 0
        heapq.heappush(open_set, (0.0, counter, start))

        came_from = {}
        g_score = {start: 0.0}

        while open_set:
            _, _, curr = heapq.heappop(open_set)

            if curr == goal:
                return self.reconstruct(came_from, curr)

            cc, cr = curr

            for dc, dr, base_cost in self.DIRS:
                nc, nr = cc + dc, cr + dr

                if not (0 <= nc < w and 0 <= nr < h):
                    continue

                if occ_grid[nr, nc]:
                    continue

                dist = dist_grid[nr, nc]

                obstacle_cost = 0.0
                if dist < SAFE_DISTANCE:
                    d = max(dist, ROBOT_RADIUS)
                    obstacle_cost = OBSTACLE_COST_GAIN * ((SAFE_DISTANCE - d) / SAFE_DISTANCE) ** 2

                new_cost = g_score[curr] + base_cost + obstacle_cost
                nxt = (nc, nr)

                if new_cost < g_score.get(nxt, float('inf')):
                    g_score[nxt] = new_cost
                    f = new_cost + self.heuristic(nxt, goal)
                    counter += 1
                    came_from[nxt] = curr
                    heapq.heappush(open_set, (f, counter, nxt))

        return None

    def heuristic(self, a, b):
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        return max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)

    def find_nearest_free(self, grid, pos, radius=10):
        cx, cy = pos
        h, w = grid.shape
        for r in range(1, radius + 1):
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    nx, ny = cx + dx, cy + dy
                    if 0 <= nx < w and 0 <= ny < h:
                        if not grid[ny, nx]:
                            return (nx, ny)
        return None

    def reconstruct(self, came_from, curr):
        path = [curr]
        while curr in came_from:
            curr = came_from[curr]
            path.append(curr)
        return list(reversed(path))

class AStarPlanner(Node):
    def __init__(self):
        super().__init__('astar_planner')

        # State
        self.map_msg = None
        self.occ_grid = None
        self.dist_grid = None
        self.current_pose = None
        self.goal_pose = None
        self.current_path = None
        
        # Obstacle from LiDAR
        self.latest_scan = None
        self.local_obstacles = []
        self.obstacle_lock = Lock()
        
        # Replanning state
        self.last_replan_time = 0.0
        self.last_robot_pos = None
        self.last_obstacle_dist = float('inf')
        self.replan_required = False

        self.grid_builder = GridBuilder(INFLATION_RADIUS, MAP_RESOLUTION)
        self.astar = AStarCore()

        # QoS
        qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        # Subscribers
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Publisher
        self.path_pub = self.create_publisher(Path, '/path', 10)
        
        # Untuk stop robot sementara saat replan
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer untuk replanning periodik
        self.replan_timer = self.create_timer(REPLAN_INTERVAL, self.check_and_replan)

        self.get_logger().info("AStar Planner Ready - WITH DYNAMIC REPLANNING")
        self.get_logger().info(f"Replan interval: {REPLAN_INTERVAL}s")
        self.get_logger().info(f"Replan distance threshold: {REPLAN_DISTANCE_THRESHOLD}m")

    def map_callback(self, msg):
        self.map_msg = msg
        self.occ_grid, self.dist_grid = self.grid_builder.build(msg)
        self.get_logger().debug("Map updated")

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def goal_callback(self, msg):
        self.goal_pose = msg.pose
        self.get_logger().info(f"New goal: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})")
        # Immediate replan on new goal
        self.plan_and_publish()

    def scan_callback(self, msg):
        self.latest_scan = msg
        self.detect_new_obstacles(msg)

    def detect_new_obstacles(self, scan):
        """Deteksi obstacle baru dari LiDAR yang tidak ada di map"""
        if self.current_pose is None:
            return
        
        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y
        robot_yaw = self.get_yaw()
        
        new_obstacles = []
        
        angle = scan.angle_min
        for i, range_val in enumerate(scan.ranges):
            if 0.1 < range_val < 3.0:  # obstacle dalam 3 meter
                # Hitung posisi obstacle di world frame
                obstacle_angle = robot_yaw + angle
                ox = robot_x + range_val * math.cos(obstacle_angle)
                oy = robot_y + range_val * math.sin(obstacle_angle)
                new_obstacles.append((ox, oy, range_val))
            
            angle += scan.angle_increment
        
        with self.obstacle_lock:
            self.local_obstacles = new_obstacles
            
            # Cek apakah ada obstacle baru yang menghalangi path
            if self.current_path and len(self.local_obstacles) > 0:
                min_dist = self.check_path_obstacle_distance()
                if min_dist < ROBOT_RADIUS + 0.1:  # Obstacle terlalu dekat dengan path
                    self.replan_required = True
                    self.get_logger().warn(f"Obstacle detected near path! Distance: {min_dist:.2f}m")

    def get_yaw(self):
        if self.current_pose is None:
            return 0.0
        q = self.current_pose.orientation
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def check_path_obstacle_distance(self):
        """Cek jarak terdekat antara path dan obstacle baru"""
        if self.current_path is None or len(self.local_obstacles) == 0:
            return float('inf')
        
        min_dist = float('inf')
        
        for ox, oy, _ in self.local_obstacles:
            for pose in self.current_path:
                px = pose.pose.position.x
                py = pose.pose.position.y
                dist = math.hypot(ox - px, oy - py)
                if dist < min_dist:
                    min_dist = dist
        
        return min_dist

    def check_deviation_from_path(self):
        """Cek apakah robot menyimpang dari path yang direncanakan"""
        if self.current_path is None or self.current_pose is None or len(self.current_path) < 2:
            return False
        
        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y
        
        # Cari jarak terdekat ke path
        min_dist = float('inf')
        for pose in self.current_path:
            px = pose.pose.position.x
            py = pose.pose.position.y
            dist = math.hypot(robot_x - px, robot_y - py)
            if dist < min_dist:
                min_dist = dist
        
        if min_dist > REPLAN_DISTANCE_THRESHOLD:
            self.get_logger().warn(f"Robot deviated from path! Distance: {min_dist:.2f}m")
            return True
        
        return False

    def check_robot_movement(self):
        """Cek apakah robot sudah bergerak cukup jauh untuk replan"""
        if self.last_robot_pos is None or self.current_pose is None:
            self.last_robot_pos = (self.current_pose.position.x, self.current_pose.position.y) if self.current_pose else None
            return False
        
        dx = self.current_pose.position.x - self.last_robot_pos[0]
        dy = self.current_pose.position.y - self.last_robot_pos[1]
        moved = math.hypot(dx, dy)
        
        if moved > 0.3:  # Bergerak 30cm, update posisi referensi
            self.last_robot_pos = (self.current_pose.position.x, self.current_pose.position.y)
            return True
        
        return False

    def check_and_replan(self):
        """Periodik cek apakah perlu replanning"""
        if self.goal_pose is None or self.map_msg is None or self.current_pose is None:
            return
        
        # Kondisi replanning:
        # 1. Ada obstacle baru menghalangi path
        # 2. Robot menyimpang dari path
        # 3. Path obstacle terlalu dekat
        
        should_replan = False
        
        if self.replan_required:
            should_replan = True
            self.replan_required = False
        
        if self.check_deviation_from_path():
            should_replan = True
        
        # Cek apakah goal masih bisa dicapai dengan obstacle baru
        if self.is_goal_blocked():
            should_replan = True
            self.get_logger().warn("Goal is blocked by obstacle! Replanning...")
        
        if should_replan:
            self.stop_robot()
            self.plan_and_publish()

    def is_goal_blocked(self):
        """Cek apakah goal terblokir obstacle berdasarkan scan terbaru"""
        if self.goal_pose is None or self.latest_scan is None or self.current_pose is None:
            return False
        
        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y
        goal_x = self.goal_pose.position.x
        goal_y = self.goal_pose.position.y
        
        # Hitung arah ke goal
        angle_to_goal = math.atan2(goal_y - robot_y, goal_x - robot_x)
        robot_yaw = self.get_yaw()
        relative_angle = angle_to_goal - robot_yaw
        
        # Cek scan ke arah goal
        angle = self.latest_scan.angle_min
        for i, range_val in enumerate(self.latest_scan.ranges):
            if abs(angle - relative_angle) < math.radians(15):
                if 0.1 < range_val < 1.0:  # Obstacle dalam 1 meter ke arah goal
                    dist_to_goal = math.hypot(goal_x - robot_x, goal_y - robot_y)
                    if range_val < dist_to_goal - ROBOT_RADIUS:
                        return True
            angle += self.latest_scan.angle_increment
        
        return False

    def stop_robot(self):
        """Stop robot sementara saat replanning"""
        cmd = Twist()
        self.cmd_pub.publish(cmd)
        self.get_logger().debug("Robot stopped for replanning")

    def plan_and_publish(self):
        """Plan ulang dan publish path baru"""
        if self.map_msg is None or self.current_pose is None or self.goal_pose is None:
            return
        
        origin = self.map_msg.info.origin
        
        start = self.grid_builder.world_to_grid(
            self.current_pose.position.x,
            self.current_pose.position.y,
            origin
        )
        
        goal = self.grid_builder.world_to_grid(
            self.goal_pose.position.x,
            self.goal_pose.position.y,
            origin
        )
        
        # Update grid dengan obstacle dari LiDAR
        self.update_grid_with_lidar_obstacles()
        
        # Plan path
        path = self.astar.plan(self.occ_grid, self.dist_grid, start, goal)
        
        if path is None:
            self.get_logger().warn("Path planning failed - trying alternative goal")
            # Cari goal terdekat yang accessible
            alternative_goal = self.find_accessible_goal(goal, start)
            if alternative_goal:
                path = self.astar.plan(self.occ_grid, self.dist_grid, start, alternative_goal)
        
        if path is None:
            self.get_logger().error("No valid path found!")
            return
        
        world_path = []
        for c, r in path:
            wx, wy = self.grid_builder.grid_to_world(c, r, origin)
            world_path.append((wx, wy))
        
        # Smooth path
        if len(world_path) > 3:
            world_path = self.smooth_path(world_path)
        
        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        
        for x, y in world_path:
            p = PoseStamped()
            p.header.frame_id = 'map'
            p.pose.position.x = x
            p.pose.position.y = y
            p.pose.orientation.w = 1.0
            msg.poses.append(p)
        
        self.current_path = msg.poses
        self.path_pub.publish(msg)
        self.last_replan_time = self.get_clock().now().nanoseconds / 1e9
        
        self.get_logger().info(f"New path published: {len(world_path)} waypoints")

    def update_grid_with_lidar_obstacles(self):
        """Tambahkan obstacle dari LiDAR ke occupancy grid"""
        if self.occ_grid is None or len(self.local_obstacles) == 0:
            return
        
        h, w = self.occ_grid.shape
        origin = self.map_msg.info.origin
        
        for ox, oy, _ in self.local_obstacles:
            col, row = self.grid_builder.world_to_grid(ox, oy, origin)
            if 0 <= col < w and 0 <= row < h:
                # Inflate obstacle di grid
                radius_cells = int(INFLATION_RADIUS / MAP_RESOLUTION)
                for dr in range(-radius_cells, radius_cells + 1):
                    for dc in range(-radius_cells, radius_cells + 1):
                        nr, nc = row + dr, col + dc
                        if 0 <= nr < h and 0 <= nc < w:
                            if dr*dr + dc*dc <= radius_cells*radius_cells:
                                self.occ_grid[nr, nc] = True
                                self.dist_grid[nr, nc] = 0.0
        
        self.get_logger().debug(f"Updated grid with {len(self.local_obstacles)} LiDAR obstacles")

    def find_accessible_goal(self, original_goal, start, max_radius=20):
        """Cari titik goal alternatif yang accessible dalam radius tertentu"""
        h, w = self.occ_grid.shape
        gx, gy = original_goal
        
        for r in range(1, max_radius + 1):
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    nx, ny = gx + dx, gy + dy
                    if 0 <= nx < w and 0 <= ny < h:
                        if not self.occ_grid[ny, nx]:
                            # Cek apakah ada path ke titik ini
                            path = self.astar.plan(self.occ_grid, self.dist_grid, start, (nx, ny))
                            if path:
                                self.get_logger().info(f"Found alternative goal at ({nx}, {ny})")
                                return (nx, ny)
        return None

    def smooth_path(self, path, iterations=3):
        path_arr = np.array(path)
        for _ in range(iterations):
            new_path = path_arr.copy()
            for i in range(1, len(path_arr) - 1):
                new_path[i] = (path_arr[i-1] + path_arr[i] + path_arr[i+1]) / 3.0
            path_arr = new_path
        return [tuple(p) for p in path_arr]

def main(args=None):
    rclpy.init(args=args)
    node = AStarPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()