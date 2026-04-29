"""
sim.launch.py — Nexus Omni 4WD Simulation Launch (STABLE FINAL)
==================================================================

PERBAIKAN FINAL:

✔ FIX #1 — GAZEBO_MODEL_PATH bersih (tidak ambil dari environment kotor)
✔ FIX #2 — Timing sinkron (tidak race condition)
✔ FIX #3 — Spawn robot setelah world siap
✔ FIX #4 — gzclient connect setelah semua ready

URUTAN EKSEKUSI:
  0s   → gzserver start
  5s   → spawn robot
  10s  → gzclient start

Usage:
  ros2 launch gazebo_sim sim.launch.py
  ros2 launch gazebo_sim sim.launch.py headless:=true
"""

import os
import subprocess

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    TimerAction,
    LogInfo,
)
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ================================================================
    # PATHS
    # ================================================================
    gazebo_sim_pkg = get_package_share_directory('gazebo_sim')
    robot_desc_pkg = get_package_share_directory('robot_description')

    world_file = os.path.join(
        gazebo_sim_pkg, 'worlds', 'indoor_obstacles.world'
    )
    urdf_file = os.path.join(
        robot_desc_pkg, 'urdf', 'robot.urdf.xacro'
    )

    # ================================================================
    # ARGUMENTS
    # ================================================================
    declare_headless = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='true = tanpa gzclient GUI'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true'
    )

    headless     = LaunchConfiguration('headless')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ================================================================
    # XACRO → URDF
    # ================================================================
    try:
        robot_description_content = subprocess.run(
            ['xacro', urdf_file],
            capture_output=True,
            text=True,
            check=True
        ).stdout
    except subprocess.CalledProcessError as e:
        print(f'[ERROR] xacro gagal: {e.stderr}')
        robot_description_content = ''
    except FileNotFoundError:
        print('[ERROR] xacro tidak ditemukan (install ros-humble-xacro)')
        robot_description_content = ''

    # ================================================================
    # ENVIRONMENT (BERSIH — TANPA PATH ROS)
    # ================================================================
    sim_env = {
        'GAZEBO_MODEL_PATH': ':'.join([
            '/usr/share/gazebo-11/models',
            os.path.join(os.path.expanduser('~'), '.gazebo', 'models'),
        ]),
        'OGRE_RTT_MODE': 'Copy',
    }

    # ================================================================
    # GZSERVER
    # ================================================================
    gzserver = ExecuteProcess(
        cmd=[
            'gzserver',
            '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            world_file,
        ],
        additional_env=sim_env,
        output='screen',
        sigterm_timeout='10',
    )

    # ================================================================
    # ROBOT STATE PUBLISHER
    # ================================================================
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time,
        }],
        output='screen',
    )

    # ================================================================
    # SPAWN ROBOT (DELAY 5 DETIK)
    # ================================================================
    spawn_robot = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_nexus_robot',
                arguments=[
                    '-topic', 'robot_description',
                    '-entity', 'nexus_robot',
                    '-x', '-3.0',
                    '-y', '-3.0',
                    '-z', '0.071',
                    '-R', '0', '-P', '0', '-Y', '0',
                ],
                output='screen',
            )
        ]
    )

    # ================================================================
    # GZCLIENT (DELAY 10 DETIK)
    # ================================================================
    gzclient = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'gzclient',
                    '--verbose',
                    '--render-engine', 'ogre'
                ],
                additional_env=sim_env,
                output='screen',
                condition=UnlessCondition(headless),
            )
        ]
    )

    # ================================================================
    # INFO
    # ================================================================
    startup_info = LogInfo(
        msg=(
            '\n================================================\n'
            ' Nexus Omni Simulation (FINAL STABLE)\n'
            '================================================\n'
            ' Debug:\n'
            '   ros2 topic echo /odom --once\n'
            '   ros2 topic list | grep -E "cmd_vel|scan|odom"\n\n'
            ' Jika GUI kosong:\n'
            '   Tekan F (focus)\n'
            '   Scroll zoom\n\n'
            ' Headless:\n'
            '   ros2 launch gazebo_sim sim.launch.py headless:=true\n'
            '================================================'
        )
    )

    return LaunchDescription([
        declare_headless,
        declare_use_sim_time,
        startup_info,
        gzserver,
        robot_state_publisher,
        spawn_robot,
        gzclient,
    ])