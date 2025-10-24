#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap, Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='True')
    map_arg     = DeclareLaunchArgument('map', default_value='')

    nav2_pkg       = get_package_share_directory('nav2_bringup')
    bringup_launch = os.path.join(nav2_pkg, 'launch', 'bringup_launch.py')
    nav2_params    = os.path.join(nav2_pkg, 'params', 'nav2_params.yaml')

    supervisor = Node(
        package='tb3_supervisor',
        executable='supervisor_node',
        name='supervisor',
        output='screen',
        parameters=[{
            'scan_topic': '/scan',
            'input_cmd_topic': '/cmd_vel_raw',
            'output_cmd_topic': '/cmd_vel',
            'slow_down_range': 0.60,
            'stop_range': 0.30,
            'fov_deg': 60.0,
            'scale_angular': False
        }]
    )

    def _start_nav2(context, *args, **kwargs):
        map_path = LaunchConfiguration('map').perform(context)
        actions = []
        actions.append(LogInfo(msg=f"[nav2_supervised] Incoming map:= '{map_path}'"))

        # Nav2 publishes /cmd_vel -> remapped to /cmd_vel_raw, supervisor publishes final /cmd_vel
        actions.append(SetRemap(src='/cmd_vel', dst='/cmd_vel_raw'))

        launch_args = {
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': nav2_params,
            'autostart': 'True',
        }
        if map_path and os.path.isfile(map_path):
            actions.append(LogInfo(msg=f"[nav2_supervised] Passing map to bringup: '{map_path}'"))
            launch_args['map'] = map_path
        else:
            if map_path:
                actions.append(LogInfo(msg=f"[nav2_supervised] WARNING: Map not found: '{map_path}'. Starting Nav2 without map."))

        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_launch),
            launch_arguments=launch_args.items()
        ))
        actions.append(supervisor)
        return actions

    return LaunchDescription([
        use_sim_time,
        map_arg,
        OpaqueFunction(function=_start_nav2),
    ])