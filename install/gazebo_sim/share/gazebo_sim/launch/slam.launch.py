from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': True,
        }]
    )

    return LaunchDescription([
        slam_toolbox
    ])