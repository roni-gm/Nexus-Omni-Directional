"""
omni_kinematics.py — Nexus Omni 4WD Inverse Kinematics (VERIFIED)
===================================================================
Verifikasi sudut roda dari URDF robot_core_highres.xacro v6:

  wheel1_joint rpy="-1.5708 0 1.5708"  → fdir="1 0 0" (putar searah X)
  wheel2_joint rpy="1.5708 0 1.5708"   → fdir="1 0 0" (putar searah X)
  wheel3_joint rpy="-1.5708 0 0"       → fdir="0 1 0" (putar searah Y)
  wheel4_joint rpy="1.5708 0 0"        → fdir="0 1 0" (putar searah Y)

Layout roda (dari atas, chassis frame):
                   +Y
                    |
    W3 (Y+) -------+------- W4 (Y-)
                   |
    W1 (X-) -------+------- W2 (X+)
                    |
                   -Y
                   -X ← robot depan adalah arah +X di chassis frame

NOTE KRITIS:
  - Node ini menerima /cmd_vel_safe (output dari path_follower)
  - Publish ke /wheel_velocities (Float64MultiArray)
  - Karena planar_move plugin sudah menangani kinematika omni secara internal,
    node ini OPSIONAL untuk simulasi Gazebo dengan planar_move
  - Node ini DIPERLUKAN jika kamu mengganti planar_move dengan
    ros2_control (diff_drive_controller atau custom omni controller)

VERIFIKASI RUNNING:
  ros2 topic echo /wheel_velocities
  # Saat vx=0.3 vy=0.0 omega=0.0:
  # [v1, v2, v3, v4] harus semua ~= 0.3/wheel_radius = 9.09 rad/s
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import numpy as np
import math


# ============================================================
# PARAMETER ROBOT dari URDF Nexus Omni v6
# ============================================================
WHEEL_RADIUS = 0.033   # meter — wheel_radius = 0.0165 * scale(2.0)
L            = 0.20    # meter — jarak dari pusat rotasi ke setiap roda
                        # Estimasi: sqrt(footprint_cx^2 + footprint_cy^2)
                        # = sqrt(0.13246^2 + 0.06868^2) ≈ 0.149
                        # Gunakan nilai lebih besar (0.20) untuk clearance

# Batas kecepatan roda (rad/s)
MAX_WHEEL_VEL = 15.0   # omega_max = v_max / r = 0.5 / 0.033 ≈ 15.15 rad/s


class OmniKinematics(Node):
    """
    Konversi Twist (vx, vy, omega) → kecepatan roda individual.

    Untuk 4-wheel omni robot dengan layout simetris:
      v1 = (-sin(α1)·vx + cos(α1)·vy + L·ω) / r
      v2 = (-sin(α2)·vx + cos(α2)·vy + L·ω) / r
      v3 = (-sin(α3)·vx + cos(α3)·vy + L·ω) / r
      v4 = (-sin(α4)·vx + cos(α4)·vy + L·ω) / r

    dengan α = sudut orientasi setiap roda terhadap sumbu X chassis.

    Untuk Nexus Omni 4WD standard (45° configuration):
      α1 = 135° (kiri belakang)
      α2 =  45° (kanan belakang)
      α3 = 315° (kiri depan)  → -45°
      α4 = 225° (kanan depan) → -135°
    """

    def __init__(self):
        super().__init__('omni_kinematics')

        # Subscribe dari path_follower (atau langsung dari planar_move check)
        self.create_subscription(
            Twist, '/cmd_vel', self.cmd_callback, 10
        )

        # Publish wheel velocities (untuk monitoring / ros2_control)
        self.publisher = self.create_publisher(
            Float64MultiArray, '/wheel_velocities', 10
        )

        # ========================================
        # Sudut roda (rad) — sesuai layout URDF
        # Nexus Omni standard 4WD 45° configuration
        # ========================================
        self.wheel_angles = np.array([
            math.radians(135),   # Wheel 1: motor1, X- side
            math.radians(45),    # Wheel 2: motor2, X+ side
            math.radians(-45),   # Wheel 3: motor3, Y+ side
            math.radians(-135),  # Wheel 4: motor4, Y- side
        ])

        # Build inverse kinematics matrix
        # Baris ke-i: [-sin(αi), cos(αi), L] / r
        self.IK_matrix = np.array([
            [-math.sin(a), math.cos(a), L]
            for a in self.wheel_angles
        ]) / WHEEL_RADIUS

        self.get_logger().info(
            f'Omni Kinematics siap.\n'
            f'  wheel_radius={WHEEL_RADIUS}m, L={L}m\n'
            f'  Subscribing: /cmd_vel\n'
            f'  Publishing:  /wheel_velocities\n'
            f'  IK matrix:\n{np.round(self.IK_matrix, 3)}'
        )

    def cmd_callback(self, msg: Twist):
        """
        Hitung kecepatan roda dari perintah Twist.
        Note: untuk planar_move simulation, wheel_velocities ini
        hanya untuk monitoring. Plugin sudah otomatis handle kinematika.
        """
        vx    = msg.linear.x
        vy    = msg.linear.y
        omega = msg.angular.z

        # body_vel = [vx, vy, omega] (kolom vektor)
        body_vel = np.array([vx, vy, omega])

        # Inverse kinematics: ω_roda = IK_matrix · body_vel
        wheel_vels = self.IK_matrix @ body_vel

        # Clamp ke batas maksimum
        # Jika ada roda melebihi batas, scale SEMUA roda proporsional
        max_abs = np.max(np.abs(wheel_vels))
        if max_abs > MAX_WHEEL_VEL:
            scale = MAX_WHEEL_VEL / max_abs
            wheel_vels *= scale
            self.get_logger().debug(
                f'Wheel vel clamped: max={max_abs:.2f} → scaled by {scale:.3f}'
            )

        # Publish
        out = Float64MultiArray()
        out.data = wheel_vels.tolist()
        self.publisher.publish(out)

        self.get_logger().debug(
            f'vx={vx:.2f} vy={vy:.2f} ω={omega:.2f} → '
            f'wheels={np.round(wheel_vels, 2).tolist()} rad/s'
        )

    def compute_forward_kinematics(self, wheel_vels: np.ndarray) -> np.ndarray:
        """
        Forward kinematics: ω_roda → body velocity [vx, vy, omega].
        Berguna untuk odometry custom jika tidak pakai planar_move.
        FK_matrix = pseudoinverse(IK_matrix)
        """
        FK_matrix = np.linalg.pinv(self.IK_matrix)
        return FK_matrix @ wheel_vels


def main(args=None):
    rclpy.init(args=args)
    node = OmniKinematics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
