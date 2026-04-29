import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

import math


# ==============================
# PARAMETERS (FINAL STABLE)
# ==============================

LOOKAHEAD_DIST = 0.7

MAX_LINEAR_VEL  = 0.4
MAX_LATERAL_VEL = 0.35
MAX_ANGULAR_VEL = 0.8

K_ANGULAR = 1.0

GOAL_REACH_DIST = 0.25

# SAFETY
STOP_DISTANCE     = 0.25
SLOW_DISTANCE     = 0.5
CRITICAL_DISTANCE = 0.15

CMD_SMOOTHING = 0.2


# ==============================
# PURE PURSUIT
# ==============================

class PurePursuit:

    def __init__(self, lookahead_dist):
        self.lookahead_dist = lookahead_dist

    def get_lookahead_point(self, rx, ry, path):

        best_point = None
        best_progress = -1.0

        for i in range(len(path) - 1):
            p1 = path[i].pose.position
            p2 = path[i+1].pose.position

            dx = p2.x - p1.x
            dy = p2.y - p1.y

            seg_len = math.hypot(dx, dy)
            if seg_len < 1e-6:
                continue

            fx = p1.x - rx
            fy = p1.y - ry

            a = dx*dx + dy*dy
            b = 2.0 * (fx*dx + fy*dy)
            c = fx*fx + fy*fy - self.lookahead_dist**2

            disc = b*b - 4.0*a*c
            if disc < 0:
                continue

            sqrt_disc = math.sqrt(disc)

            t1 = (-b - sqrt_disc) / (2.0 * a)
            t2 = (-b + sqrt_disc) / (2.0 * a)

            for t in [t2, t1]:
                if 0.0 <= t <= 1.0:
                    progress = i + t
                    if progress > best_progress:
                        best_progress = progress
                        best_point = (p1.x + t*dx, p1.y + t*dy)

        if best_point is None:
            last = path[-1].pose.position
            return (last.x, last.y)

        return best_point


# ==============================
# NODE
# ==============================

class PathFollower(Node):

    def __init__(self):
        super().__init__('path_follower')

        self.path = []
        self.current_pose = None
        self.scan = None

        self.prev_cmd = Twist()
        self.goal_reached = False   # 🔥 FIX UTAMA

        self.pure_pursuit = PurePursuit(LOOKAHEAD_DIST)

        self.create_subscription(Path, '/path', self.path_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info("🔥 Path Follower FINAL PERFECT")

    # ==============================
    # CALLBACKS
    # ==============================

    def path_callback(self, msg):
        self.path = list(msg.poses)
        self.goal_reached = False  # reset kalau path baru
        self.get_logger().info(f"Path diterima: {len(self.path)}")

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def scan_callback(self, msg):
        self.scan = msg

    # ==============================
    # UTIL
    # ==============================

    def get_yaw(self):
        q = self.current_pose.orientation
        siny = 2.0 * (q.w*q.z + q.x*q.y)
        cosy = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        return math.atan2(siny, cosy)

    def get_front_distance(self):

        if self.scan is None:
            return float('inf')

        ranges = self.scan.ranges
        n = len(ranges)

        front = ranges[int(n*0.4):int(n*0.6)]

        valid = [r for r in front if not math.isinf(r) and not math.isnan(r)]

        if not valid:
            return float('inf')

        return min(valid)

    def hard_stop(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)
        self.prev_cmd = Twist()

    # ==============================
    # MAIN LOOP
    # ==============================

    def control_loop(self):

        if self.current_pose is None:
            return

        # 🔥 HARD LOCK (ANTI GOYANG)
        if self.goal_reached:
            self.cmd_pub.publish(Twist())
            return

        if not self.path:
            return

        rx = self.current_pose.position.x
        ry = self.current_pose.position.y
        yaw = self.get_yaw()

        goal = self.path[-1].pose.position
        dist_to_goal = math.hypot(goal.x - rx, goal.y - ry)

        # =========================
        # GOAL CHECK (FINAL FIX)
        # =========================
        if dist_to_goal < GOAL_REACH_DIST:

            if not self.goal_reached:
                self.get_logger().info("🎯 Goal tercapai — HARD STOP")

            self.goal_reached = True
            self.path = []

            self.hard_stop()
            return

        # =========================
        # OBSTACLE SAFETY
        # =========================
        front_dist = self.get_front_distance()

        if front_dist < CRITICAL_DISTANCE:
            self.get_logger().warn("Obstacle sangat dekat → STOP")
            self.hard_stop()
            return

        slow_factor = 1.0
        if front_dist < SLOW_DISTANCE:
            slow_factor = (front_dist - STOP_DISTANCE) / (SLOW_DISTANCE - STOP_DISTANCE)
            slow_factor = max(0.3, min(1.0, slow_factor))

        # =========================
        # PURE PURSUIT
        # =========================
        lx, ly = self.pure_pursuit.get_lookahead_point(rx, ry, self.path)

        dx = lx - rx
        dy = ly - ry

        dist = math.hypot(dx, dy)

        # 🔥 ANTI MICRO MOVEMENT
        if dist < 0.08:
            self.hard_stop()
            return

        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)

        vx =  cos_y * dx + sin_y * dy
        vy = -sin_y * dx + cos_y * dy

        scale = min(1.0, dist / LOOKAHEAD_DIST)
        vx *= scale
        vy *= scale

        vx = max(min(vx, MAX_LINEAR_VEL), -MAX_LINEAR_VEL)
        vy = max(min(vy, MAX_LATERAL_VEL), -MAX_LATERAL_VEL)

        target_angle = math.atan2(dy, dx)
        angle_error = math.atan2(math.sin(target_angle - yaw),
                                 math.cos(target_angle - yaw))

        omega = K_ANGULAR * angle_error
        omega = max(min(omega, MAX_ANGULAR_VEL), -MAX_ANGULAR_VEL)

        # slow near obstacle
        vx *= slow_factor
        vy *= slow_factor
        omega *= slow_factor

        # =========================
        # SMOOTHING
        # =========================
        cmd = Twist()
        cmd.linear.x = (1-CMD_SMOOTHING)*self.prev_cmd.linear.x + CMD_SMOOTHING*vx
        cmd.linear.y = (1-CMD_SMOOTHING)*self.prev_cmd.linear.y + CMD_SMOOTHING*vy
        cmd.angular.z = (1-CMD_SMOOTHING)*self.prev_cmd.angular.z + CMD_SMOOTHING*omega

        self.prev_cmd = cmd

        self.cmd_pub.publish(cmd)


# ==============================
# MAIN
# ==============================

def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()