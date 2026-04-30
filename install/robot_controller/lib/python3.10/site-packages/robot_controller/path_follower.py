# path_follower.py - FINAL ULTRA PRECISE VERSION
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan

import math
import numpy as np
from collections import deque

# ===================================================
# PARAMETER FINAL - TOLERANSI DIPERKECIL
# ===================================================

# Pure Pursuit
LOOKAHEAD_DIST = 0.35           # meter - lookahead point
LOOKAHEAD_MIN = 0.25             # meter - minimal lookahead pas belok
LOOKAHEAD_MAX = 0.50             # meter - maksimal lookahead saat lurus

# Velocity limits
MAX_LINEAR_VEL  = 0.40          # m/s
MAX_LATERAL_VEL = 0.35          # m/s  
MAX_ANGULAR_VEL = 1.2           # rad/s

# Control gains (DIPERBESAR untuk presisi)
K_LINEAR  = 1.5                 # gain linear velocity
K_ANGULAR = 1.8                 # gain angular velocity
K_LATERAL = 1.2                 # gain lateral tracking

# Tracking parameters (TOLERANSI DIPERKECIL)
WAYPOINT_REACH_DIST = 0.05      # meter - 5cm (sebelumnya 10cm)
GOAL_REACH_DIST = 0.08          # meter - 8cm (sebelumnya 12cm)
PATH_DEVIATION_THRESHOLD = 0.10 # meter - 10cm (sebelumnya 15cm)

# PID Controller gains
PID_KP = 1.2                    # Proportional
PID_KI = 0.05                   # Integral  
PID_KD = 0.8                    # Derivative

# Obstacle avoidance (TOLERANSI DIPERKECIL)
ROBOT_RADIUS = 0.16             # meter - lebih kecil untuk lorong sempit
STOP_DISTANCE = 0.10            # meter - 10cm stop (sebelumnya 12cm)
SLOW_DISTANCE = 0.25            # meter - 25cm mulai slow (sebelumnya 25cm)
OBSTACLE_LATERAL_FORCE = 0.3    # gaya lateral saat hindari obstacle

# Motion constraints
MAX_ACCEL = 1.0                 # m/s^2 (respons lebih cepat)
MAX_DECEL = 1.5                 # m/s^2
MAX_ANGULAR_ACCEL = 2.5         # rad/s^2

class PIDController:
    def __init__(self, kp, ki, kd, dt=0.05):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.integral = 0.0
        self.prev_error = 0.0
    
    def compute(self, error):
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        
        output = (self.kp * error + 
                  self.ki * self.integral + 
                  self.kd * derivative)
        
        # Anti-windup
        self.integral = max(-0.5, min(0.5, self.integral))
        self.prev_error = error
        
        return max(-1.0, min(1.0, output))

class PurePursuitController:
    def __init__(self):
        self.lookahead_dist = LOOKAHEAD_DIST
        self.prev_lookahead = None
    
    def get_adaptive_lookahead(self, speed, curvature):
        """Adaptive lookahead berdasarkan kecepatan dan tikungan"""
        if speed < 0.05:
            return LOOKAHEAD_MIN
        
        # Lebih kecil lookahead saat belok tajam
        turn_factor = 1.0 / (1.0 + abs(curvature) * 3.0)
        
        # Lebih besar lookahead saat kecepatan tinggi
        speed_factor = min(1.0, speed / MAX_LINEAR_VEL)
        
        lookahead = LOOKAHEAD_MIN + (LOOKAHEAD_MAX - LOOKAHEAD_MIN) * speed_factor * turn_factor
        return lookahead

    def get_lookahead_point(self, robot_x, robot_y, robot_yaw, path):
        if len(path) < 2:
            return None, None
        
        # Get current speed for adaptive lookahead
        current_speed = 0.0  # will be updated from caller
        
        best_point = None
        best_progress = -1.0
        best_dist = float('inf')
        
        for i in range(len(path) - 1):
            p1 = path[i].pose.position
            p2 = path[i+1].pose.position
            
            dx = p2.x - p1.x
            dy = p2.y - p1.y
            seg_len = math.hypot(dx, dy)
            
            if seg_len < 0.001:
                continue
            
            # Check distance from robot to segment
            fx = p1.x - robot_x
            fy = p1.y - robot_y
            
            t = -(fx*dx + fy*dy) / (dx*dx + dy*dy)
            t = max(0.0, min(1.0, t))
            
            closest_x = p1.x + t*dx
            closest_y = p1.y + t*dy
            dist_to_seg = math.hypot(closest_x - robot_x, closest_y - robot_y)
            
            # Look for point at lookahead distance
            remaining = self.lookahead_dist
            remaining -= dist_to_seg
            
            if remaining > 0 and t < 1.0:
                # Project forward along segment
                remaining_pct = remaining / seg_len
                if t + remaining_pct <= 1.0:
                    point_x = p1.x + (t + remaining_pct) * dx
                    point_y = p1.y + (t + remaining_pct) * dy
                    
                    progress = i + (t + remaining_pct)
                    if progress > best_progress:
                        best_progress = progress
                        best_point = (point_x, point_y)
                        best_dist = remaining
                else:
                    # Continue to next segment
                    pass
        
        if best_point is None:
            # Fallback: use endpoint
            last = path[-1].pose.position
            return last.x, last.y
        
        return best_point[0], best_point[1]

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')

        # State
        self.path = []
        self.current_pose = None
        self.current_vel = None
        self.laser_ranges = None
        self.laser_angles = None
        
        # Tracking state
        self.goal_reached_triggered = False
        self.current_waypoint_idx = 0
        self.path_deviation_count = 0
        
        # PID controllers
        self.pid_linear = PIDController(PID_KP, PID_KI, PID_KD)
        self.pid_angular = PIDController(PID_KP * 1.5, PID_KI * 0.5, PID_KD * 1.2)
        
        # Pure pursuit
        self.pure_pursuit = PurePursuitController()
        self.prev_cmd = Twist()
        
        # Velocity smoothing
        self.cmd_history = deque(maxlen=5)
        
        # Performance monitoring
        self.loop_count = 0
        self.last_debug_time = 0.0

        # Subscribers
        self.create_subscription(Path, '/path', self.path_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Control loop - 30Hz (lebih cepat untuk presisi)
        self.timer = self.create_timer(0.033, self.control_loop)

        self.get_logger().info("=" * 50)
        self.get_logger().info("PATH FOLLOWER ULTRA PRECISE VERSION")
        self.get_logger().info(f"Waypoint reach: {WAYPOINT_REACH_DIST*100:.0f}cm")
        self.get_logger().info(f"Goal reach: {GOAL_REACH_DIST*100:.0f}cm")
        self.get_logger().info(f"Max linear: {MAX_LINEAR_VEL} m/s")
        self.get_logger().info("=" * 50)

    def path_callback(self, msg):
        if len(msg.poses) > 0:
            self.path = list(msg.poses)
            self.goal_reached_triggered = False
            self.current_waypoint_idx = 0
            self.path_deviation_count = 0
            self.get_logger().info(f"Path received: {len(self.path)} waypoints")

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        self.current_vel = msg.twist.twist

    def scan_callback(self, msg):
        self.laser_ranges = msg.ranges
        self.laser_angles = []
        angle = msg.angle_min
        while angle <= msg.angle_max:
            self.laser_angles.append(angle)
            angle += msg.angle_increment

    def get_yaw(self):
        if self.current_pose is None:
            return 0.0
        q = self.current_pose.orientation
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def get_current_speed(self):
        if self.current_vel is None:
            return 0.0
        return math.hypot(self.current_vel.linear.x, self.current_vel.linear.y)

    def get_obstacle_info(self):
        """Dapatkan informasi obstacle dengan lebih detail"""
        if self.laser_ranges is None:
            return float('inf'), 0.0, 0.0
        
        min_dist = float('inf')
        min_angle = 0.0
        closest_side = 0.0  # -1 left, 0 front, 1 right
        
        for i, r in enumerate(self.laser_ranges):
            if not (0.1 < r < 5.0):
                continue
            
            angle = self.laser_angles[i]
            
            # Check front 120 degrees
            if abs(angle) < math.radians(60):
                dist = r
                if dist < min_dist:
                    min_dist = dist
                    min_angle = angle
                    
                    # Determine side
                    if angle < -math.radians(20):
                        closest_side = -1.0  # left
                    elif angle > math.radians(20):
                        closest_side = 1.0   # right
                    else:
                        closest_side = 0.0   # front
        
        return min_dist, min_angle, closest_side

    def find_closest_waypoint(self):
        """Cari waypoint terdekat dengan robot (untuk pruning)"""
        if self.current_pose is None or len(self.path) == 0:
            return 0
        
        min_dist = float('inf')
        min_idx = 0
        
        for i, pose in enumerate(self.path):
            dx = pose.pose.position.x - self.current_pose.position.x
            dy = pose.pose.position.y - self.current_pose.position.y
            dist = math.hypot(dx, dy)
            
            if dist < min_dist:
                min_dist = dist
                min_idx = i
        
        return min_idx

    def prune_completed_waypoints(self):
        """Hapus waypoint yang sudah dilewati"""
        if len(self.path) <= 1:
            return
        
        closest_idx = self.find_closest_waypoint()
        
        # Only prune if very close to waypoint
        if closest_idx > 0:
            prune_to = max(0, closest_idx - 1)
            if prune_to > 0:
                self.path = self.path[prune_to:]
                self.get_logger().debug(f"Path pruned to {len(self.path)} waypoints")

    def get_path_error(self):
        """Hitung error tracking terhadap path"""
        if self.current_pose is None or len(self.path) < 2:
            return 0.0, 0.0
        
        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y
        
        closest_idx = self.find_closest_waypoint()
        if closest_idx >= len(self.path) - 1:
            closest_idx = len(self.path) - 2
        
        p1 = self.path[closest_idx].pose.position
        p2 = self.path[closest_idx + 1].pose.position
        
        # Hitung jarak tegak lurus ke segmen path
        dx = p2.x - p1.x
        dy = p2.y - p1.y
        seg_len = math.hypot(dx, dy)
        
        if seg_len < 0.001:
            return 0.0, 0.0
        
        # Vector dari p1 ke robot
        fx = robot_x - p1.x
        fy = robot_y - p1.y
        
        # Cross product untuk jarak lateral
        cross = abs(fx*dy - fy*dx) / seg_len
        
        # Projection untuk error longitudinal
        dot = (fx*dx + fy*dy) / seg_len
        lon_error = min(1.0, max(0.0, dot))
        
        # Sign: positif jika robot di kiri path
        sign = 1.0 if (fx*dy - fy*dx) > 0 else -1.0
        
        return cross * sign, lon_error

    def apply_velocity_limits(self, vx, vy, omega):
        # Limit linear speed
        linear_speed = math.hypot(vx, vy)
        if linear_speed > MAX_LINEAR_VEL:
            scale = MAX_LINEAR_VEL / linear_speed
            vx *= scale
            vy *= scale
        
        # Limit individual components
        vx = max(min(vx, MAX_LINEAR_VEL), -MAX_LINEAR_VEL)
        vy = max(min(vy, MAX_LATERAL_VEL), -MAX_LATERAL_VEL)
        omega = max(min(omega, MAX_ANGULAR_VEL), -MAX_ANGULAR_VEL)
        
        return vx, vy, omega

    def apply_smoothing(self, cmd):
        """Smoothing command untuk gerakan yang lebih halus"""
        self.cmd_history.append(cmd)
        
        if len(self.cmd_history) < 3:
            return cmd
        
        # Average filter
        smooth_cmd = Twist()
        for c in self.cmd_history:
            smooth_cmd.linear.x += c.linear.x
            smooth_cmd.linear.y += c.linear.y
            smooth_cmd.angular.z += c.angular.z
        
        len_cmd = len(self.cmd_history)
        smooth_cmd.linear.x /= len_cmd
        smooth_cmd.linear.y /= len_cmd
        smooth_cmd.angular.z /= len_cmd
        
        return smooth_cmd

    def apply_acceleration_limits(self, target_cmd):
        dt = 0.033  # 30Hz control loop
        
        # Linear acceleration
        vx_diff = target_cmd.linear.x - self.prev_cmd.linear.x
        vy_diff = target_cmd.linear.y - self.prev_cmd.linear.y
        
        max_linear_change = MAX_ACCEL * dt
        if vx_diff > max_linear_change:
            target_cmd.linear.x = self.prev_cmd.linear.x + max_linear_change
        elif vx_diff < -max_linear_change:
            target_cmd.linear.x = self.prev_cmd.linear.x - max_linear_change
        
        if vy_diff > max_linear_change:
            target_cmd.linear.y = self.prev_cmd.linear.y + max_linear_change
        elif vy_diff < -max_linear_change:
            target_cmd.linear.y = self.prev_cmd.linear.y - max_linear_change
        
        # Angular acceleration
        omega_diff = target_cmd.angular.z - self.prev_cmd.angular.z
        max_angular_change = MAX_ANGULAR_ACCEL * dt
        
        if omega_diff > max_angular_change:
            target_cmd.angular.z = self.prev_cmd.angular.z + max_angular_change
        elif omega_diff < -max_angular_change:
            target_cmd.angular.z = self.prev_cmd.angular.z - max_angular_change
        
        self.prev_cmd = target_cmd
        return target_cmd

    def control_loop(self):
        self.loop_count += 1
        
        # Validation
        if self.current_pose is None:
            return
        
        # Check goal reached
        if len(self.path) > 0 and not self.goal_reached_triggered:
            goal = self.path[-1].pose.position
            dx = goal.x - self.current_pose.position.x
            dy = goal.y - self.current_pose.position.y
            dist_to_goal = math.hypot(dx, dy)
            
            if dist_to_goal < GOAL_REACH_DIST:
                self.goal_reached_triggered = True
                self.get_logger().info(f"✅ GOAL REACHED! Distance: {dist_to_goal*100:.0f}cm")
                self.stop()
                return
        
        if self.goal_reached_triggered or len(self.path) == 0:
            return
        
        # Prune completed waypoints
        self.prune_completed_waypoints()
        
        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y
        robot_yaw = self.get_yaw()
        current_speed = self.get_current_speed()
        
        # Get obstacle info
        obstacle_dist, obstacle_angle, obstacle_side = self.get_obstacle_info()
        
        # Emergency stop
        if obstacle_dist < STOP_DISTANCE:
            self.get_logger().warn(f"EMERGENCY STOP! Obstacle at {obstacle_dist*100:.0f}cm")
            self.stop()
            return
        
        # Update adaptive lookahead
        if len(self.path) > 1:
            next_point = self.path[min(self.current_waypoint_idx + 2, len(self.path)-1)].pose.position
            dx = next_point.x - robot_x
            dy = next_point.y - robot_y
            curvature = abs(dx*math.sin(robot_yaw) - dy*math.cos(robot_yaw)) / max(0.1, math.hypot(dx, dy))
            self.pure_pursuit.lookahead_dist = self.pure_pursuit.get_adaptive_lookahead(current_speed, curvature)
        
        # Get lookahead point
        lookahead_x, lookahead_y = self.pure_pursuit.get_lookahead_point(
            robot_x, robot_y, robot_yaw, self.path
        )
        
        if lookahead_x is None:
            self.stop()
            return
        
        # Compute errors
        dx = lookahead_x - robot_x
        dy = lookahead_y - robot_y
        dist_to_lookahead = math.hypot(dx, dy)
        
        if dist_to_lookahead < 0.01:
            self.stop()
            return
        
        # Transform to robot frame
        cos_yaw = math.cos(robot_yaw)
        sin_yaw = math.sin(robot_yaw)
        
        dx_robot = cos_yaw * dx + sin_yaw * dy
        dy_robot = -sin_yaw * dx + cos_yaw * dy
        
        # Angle to target
        angle_to_target = math.atan2(dy_robot, dx_robot)
        
        # Velocity commands (Pure Pursuit)
        linear_cmd = min(K_LINEAR * dist_to_lookahead, MAX_LINEAR_VEL)
        
        # Scale berdasarkan obstacle distance
        if obstacle_dist < SLOW_DISTANCE:
            speed_factor = (obstacle_dist - STOP_DISTANCE) / (SLOW_DISTANCE - STOP_DISTANCE)
            speed_factor = max(0.3, min(1.0, speed_factor))
            linear_cmd *= speed_factor
        
        # Lateral tracking dengan PID
        lateral_error = dy_robot
        lateral_cmd = self.pid_linear.compute(lateral_error) * MAX_LATERAL_VEL
        
        # Angular tracking dengan PID  
        heading_error = angle_to_target
        angular_cmd = self.pid_angular.compute(heading_error) * MAX_ANGULAR_VEL
        
        # Obstacle avoidance lateral force
        if obstacle_dist < SLOW_DISTANCE and abs(obstacle_angle) < math.radians(30):
            # Hindari obstacle dengan bergerak ke samping
            avoid_force = OBSTACLE_LATERAL_FORCE * (1.0 - obstacle_dist / SLOW_DISTANCE)
            if obstacle_angle < 0:
                lateral_cmd += avoid_force * MAX_LATERAL_VEL  # ke kanan
            else:
                lateral_cmd -= avoid_force * MAX_LATERAL_VEL  # ke kiri
        
        # Clamp commands
        vx = max(min(linear_cmd, MAX_LINEAR_VEL), -MAX_LINEAR_VEL)
        vy = max(min(lateral_cmd, MAX_LATERAL_VEL), -MAX_LATERAL_VEL)
        omega = max(min(angular_cmd, MAX_ANGULAR_VEL), -MAX_ANGULAR_VEL)
        
        # Apply limits
        vx, vy, omega = self.apply_velocity_limits(vx, vy, omega)
        
        # Create and smooth command
        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.angular.z = omega
        
        cmd = self.apply_smoothing(cmd)
        cmd = self.apply_acceleration_limits(cmd)
        
        # Publish
        self.cmd_pub.publish(cmd)
        
        # Debug setiap 1 detik
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_debug_time > 1.0:
            self.last_debug_time = now
            path_error, _ = self.get_path_error()
            
            self.get_logger().info(
                f"📊 Vel: {current_speed:.2f}/{MAX_LINEAR_VEL} m/s | "
                f"Goal: {dist_to_goal*100:.0f}cm | "
                f"PathErr: {abs(path_error)*100:.0f}cm | "
                f"Obst: {obstacle_dist*100:.0f}cm | "
                f"Lookahead: {self.pure_pursuit.lookahead_dist*100:.0f}cm"
            )

    def stop(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)
        self.prev_cmd = cmd
        self.cmd_history.clear()

def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()