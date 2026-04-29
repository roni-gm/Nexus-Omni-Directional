import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from nav_msgs.msg import Path, OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped

import math
import heapq
import numpy as np


# ==========================
# TUNING FINAL (STABLE)
# ==========================
ROBOT_RADIUS = 0.16
SAFETY_MARGIN = 0.05
INFLATION_RADIUS = ROBOT_RADIUS + SAFETY_MARGIN

MAP_RESOLUTION = 0.07
OBSTACLE_THRESHOLD = 65


# ==========================
# GRID BUILDER
# ==========================
class GridBuilder:
    def __init__(self, inflation_radius, resolution):
        self.inflation_cells = math.ceil(inflation_radius / resolution)
        self.resolution = resolution

    def build(self, msg):
        w = msg.info.width
        h = msg.info.height
        data = np.array(msg.data).reshape((h, w))

        # 🔥 UNKNOWN TIDAK DIANGGAP OBSTACLE
        binary = (data >= OBSTACLE_THRESHOLD)

        inflated = binary.copy()

        for y in range(h):
            for x in range(w):
                if binary[y, x]:
                    for dy in range(-self.inflation_cells, self.inflation_cells + 1):
                        for dx in range(-self.inflation_cells, self.inflation_cells + 1):
                            if dx*dx + dy*dy <= self.inflation_cells**2:
                                ny, nx = y + dy, x + dx
                                if 0 <= ny < h and 0 <= nx < w:
                                    inflated[ny, nx] = True

        return inflated

    def world_to_grid(self, wx, wy, origin):
        col = int((wx - origin.position.x) / self.resolution)
        row = int((wy - origin.position.y) / self.resolution)
        return col, row

    def grid_to_world(self, col, row, origin):
        wx = col * self.resolution + origin.position.x + self.resolution / 2
        wy = row * self.resolution + origin.position.y + self.resolution / 2
        return wx, wy


# ==========================
# A* CORE
# ==========================
class AStarCore:

    DIRS = [
        (1,0,1),(-1,0,1),(0,1,1),(0,-1,1),
        (1,1,1.414),(1,-1,1.414),(-1,1,1.414),(-1,-1,1.414)
    ]

    def plan(self, grid, start, goal):
        h, w = grid.shape

        open_set = []
        heapq.heappush(open_set, (0, start))

        came_from = {}
        g = {start: 0}

        while open_set:
            _, curr = heapq.heappop(open_set)

            if curr == goal:
                return self.reconstruct(came_from, curr)

            for dc, dr, cost in self.DIRS:
                nc, nr = curr[0] + dc, curr[1] + dr

                if not (0 <= nc < w and 0 <= nr < h):
                    continue
                if grid[nr][nc]:
                    continue

                new_cost = g[curr] + cost
                if (nc, nr) not in g or new_cost < g[(nc, nr)]:
                    g[(nc, nr)] = new_cost
                    f = new_cost + self.heuristic((nc, nr), goal)
                    heapq.heappush(open_set, (f, (nc, nr)))
                    came_from[(nc, nr)] = curr

        return None

    def heuristic(self, a, b):
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        return max(dx, dy)

    def reconstruct(self, came_from, curr):
        path = [curr]
        while curr in came_from:
            curr = came_from[curr]
            path.append(curr)
        return list(reversed(path))


# ==========================
# MAIN NODE
# ==========================
class AStarPlanner(Node):

    def __init__(self):
        super().__init__('astar_planner')

        self.map_msg = None
        self.inflated_grid = None
        self.current_pose = None
        self.goal_pose = None

        self.grid_builder = GridBuilder(INFLATION_RADIUS, MAP_RESOLUTION)
        self.astar = AStarCore()

        map_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)

        self.path_pub = self.create_publisher(Path, '/path', 10)

        self.get_logger().info("A* Planner READY")

    def map_callback(self, msg):
        self.map_msg = msg
        self.inflated_grid = self.grid_builder.build(msg)

        ratio = np.sum(self.inflated_grid) / self.inflated_grid.size
        self.get_logger().info(f"Map diterima | Obstacle ratio: {ratio:.2f}")

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def goal_callback(self, msg):
        self.goal_pose = msg.pose
        self.get_logger().info("Goal diterima")
        self.plan()

    # ==========================
    # CARI TITIK BEBAS
    # ==========================
    def find_free(self, pos):
        h, w = self.inflated_grid.shape
        c, r = pos

        for radius in range(1, 20):
            for dc in range(-radius, radius + 1):
                for dr in range(-radius, radius + 1):
                    nc, nr = c + dc, r + dr
                    if 0 <= nc < w and 0 <= nr < h:
                        if not self.inflated_grid[nr][nc]:
                            return (nc, nr)
        return None

    def plan(self):

        if self.map_msg is None:
            self.get_logger().warn("Map belum ada")
            return
        if self.current_pose is None:
            self.get_logger().warn("Odom belum ada")
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

        self.get_logger().info(f"START GRID: {start}")
        self.get_logger().info(f"GOAL GRID: {goal}")

        h, w = self.inflated_grid.shape

        if not (0 <= start[0] < w and 0 <= start[1] < h):
            self.get_logger().error("Start di luar map")
            return

        if not (0 <= goal[0] < w and 0 <= goal[1] < h):
            self.get_logger().error("Goal di luar map")
            return

        # HANDLE START
        if self.inflated_grid[start[1]][start[0]]:
            self.get_logger().warn("Start kena obstacle → cari titik bebas")
            new_start = self.find_free(start)
            if new_start is None:
                self.get_logger().error("Tidak ada start bebas!")
                return
            start = new_start

        # HANDLE GOAL
        if self.inflated_grid[goal[1]][goal[0]]:
            self.get_logger().warn("Goal kena obstacle → cari titik bebas")
            new_goal = self.find_free(goal)
            if new_goal is None:
                self.get_logger().error("Tidak ada goal bebas!")
                return
            goal = new_goal

        # RUN A*
        path = self.astar.plan(self.inflated_grid, start, goal)

        if path is None:
            self.get_logger().warn("Path tidak ditemukan")
            return

        path_msg = Path()
        path_msg.header.frame_id = 'map'

        for c, r in path:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            wx, wy = self.grid_builder.grid_to_world(c, r, origin)
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        self.get_logger().info(f"Path publish: {len(path)} titik")


def main(args=None):
    rclpy.init(args=args)
    node = AStarPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()