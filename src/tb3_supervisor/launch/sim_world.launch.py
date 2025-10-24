#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # TB3 model
    set_tb3_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger')

    # Include official TurtleBot3 Gazebo world launch
    tb3_gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')
    world_launch = os.path.join(tb3_gazebo_pkg, 'launch', 'turtlebot3_world.launch.py')

    # time + spawn pose arguments
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='True')
    x_pose = DeclareLaunchArgument('x_pose', default_value='-2.0')
    y_pose = DeclareLaunchArgument('y_pose', default_value='-0.5')
    z_pose = DeclareLaunchArgument('z_pose', default_value='0.0')

    return LaunchDescription([
        set_tb3_model,
        use_sim_time, x_pose, y_pose, z_pose,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(world_launch),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'x_pose': LaunchConfiguration('x_pose'),
                'y_pose': LaunchConfiguration('y_pose'),
                'z_pose': LaunchConfiguration('z_pose'),
            }.items()
        )
    ])