import os
import glob
from setuptools import setup
 
package_name = 'gazebo_sim'
 
setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
 
        # ── WAJIB: include worlds ──────────────────────────────────────
        (os.path.join('share', package_name, 'worlds'),
            glob.glob('worlds/*.world')),
 
        # ── WAJIB: include maps (yaml + pgm) ──────────────────────────
        (os.path.join('share', package_name, 'maps'),
            glob.glob('maps/*.yaml') + glob.glob('maps/*.pgm')),
 
        # ── WAJIB: include launch files ───────────────────────────────
        (os.path.join('share', package_name, 'launch'),
            glob.glob('launch/*.launch.py')),
 
        # ── Opsional: rviz configs ────────────────────────────────────
        # (os.path.join('share', package_name, 'rviz'),
        #     glob.glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ronnigm',
    description='Gazebo simulation for Nexus Omni 4WD',
    license='MIT',
    entry_points={
        'console_scripts': [],
    },
)
