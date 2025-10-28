#!/usr/bin/env python3
# Bring up: Gazebo world + Nav2 (supervised) + RViz. Map is optional.

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, OpaqueFunction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('tb3_supervisor')

    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='True')
    map_arg = DeclareLaunchArgument('map', default_value='')
    x_pose = DeclareLaunchArgument('x_pose', default_value='-2.0')
    y_pose = DeclareLaunchArgument('y_pose', default_value='-0.5')
    z_pose = DeclareLaunchArgument('z_pose', default_value='0.0')

    sim_launch = os.path.join(pkg, 'launch', 'sim_world.launch.py')
    nav_launch = os.path.join(pkg, 'launch', 'nav2_supervised.launch.py')

    nav2_pkg = get_package_share_directory('nav2_bringup')
    rviz_config = os.path.join(nav2_pkg, 'rviz', 'nav2_default_view.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    # Decide at runtime whether to pass a map to Nav2 (works with/without a map file)
    def _include_nav2_with_valid_map(context, *args, **kwargs):
        map_path = LaunchConfiguration('map').perform(context)
        actions = []
        actions.append(LogInfo(msg=f"[bringup_all] Resolved map:= '{map_path}'"))
        launch_args = {'use_sim_time': LaunchConfiguration('use_sim_time')}
        if map_path and os.path.isfile(map_path):
            launch_args['map'] = map_path
            actions.append(LogInfo(msg=f"[bringup_all] Passing map to nav2_supervised: '{map_path}'"))
        else:
            if map_path:
                actions.append(LogInfo(msg=f"[bringup_all] WARNING: Map not found '{map_path}'. Starting without map."))
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav_launch),
            launch_arguments=launch_args.items()
        ))
        return actions

    return LaunchDescription([
        use_sim_time, map_arg,
        x_pose, y_pose, z_pose,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'x_pose': LaunchConfiguration('x_pose'),
                'y_pose': LaunchConfiguration('y_pose'),
                'z_pose': LaunchConfiguration('z_pose'),
            }.items()
        ),

        # Delay Nav2: give Gazebo time to spawn TF and topics
        TimerAction(period=5.0, actions=[OpaqueFunction(function=_include_nav2_with_valid_map)]),

        # Delay RViz slightly after Nav2 so display has data
        TimerAction(period=7.0, actions=[rviz]),
    ])
