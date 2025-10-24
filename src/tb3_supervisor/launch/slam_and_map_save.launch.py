#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='True')

    # Gazebo TurtleBot3 world
    pkg = get_package_share_directory('tb3_supervisor')
    sim_launch = os.path.join(pkg, 'launch', 'sim_world.launch.py')

    # SLAM Toolbox (online async)
    slam_pkg = get_package_share_directory('slam_toolbox')
    slam_launch = os.path.join(slam_pkg, 'launch', 'online_async_launch.py')

    # SLAM params tuned for TB3 Burger LDS-01 (~0.1m to ~3.5m)
    slam_params = {
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'min_laser_range': 0.10,   # avoid "minimum exceeds sensor" warnings
        'max_laser_range': 3.50,   # match gazebo TB3 laser plugin
    }

    # RViz (Nav2 default view)
    nav2_pkg = get_package_share_directory('nav2_bringup')
    rviz_config = os.path.join(nav2_pkg, 'rviz', 'nav2_default_view.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    #   ros2 run turtlebot3_teleop teleop_keyboard

    return LaunchDescription([
        use_sim_time,
        IncludeLaunchDescription(PythonLaunchDescriptionSource(sim_launch)),
        TimerAction(period=5.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(slam_launch),
                launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
            ),
            Node(
                package='slam_toolbox',
                executable='sync_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[slam_params]
            ),
            rviz
        ]),
    ])