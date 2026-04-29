from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():

    return LaunchDescription([

        # SLAM
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            parameters=['/home/ronnigm/astar_ws/src/gazebo_sim/config/slam.yaml'],
            output='screen'
        ),

        # A*
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='astar_planner',
                    executable='astar_planner',
                    name='astar_planner',
                    parameters=[{'use_sim_time': True}],
                    output='screen'
                )
            ]
        ),

        # Controller
        Node(
            package='robot_controller',
            executable='path_follower',
            name='path_follower',
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),
    ])