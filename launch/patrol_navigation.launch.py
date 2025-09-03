#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directories
    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    turtlebot3_navigation2_share = get_package_share_directory('turtlebot3_navigation2')
    package_share = get_package_share_directory('patrol_navigation_project_cpp')
    
    # Use TurtleBot3's default map and params for now
    default_map = os.path.join(turtlebot3_navigation2_share, 'map', 'map.yaml')
    default_params = os.path.join(nav2_bringup_share, 'params', 'nav2_params.yaml')
    
    # Declare arguments
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Full path to map yaml file'
    )
    
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Full path to param file'
    )
    
    # Include Nav2 bringup
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
            'use_sim_time': 'true'
        }.items()
    )
    
    # RViz with default nav2 config
    rviz_config = os.path.join(nav2_bringup_share, 'rviz', 'nav2_default_view.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    # Robot Patrol controller - C++ executable
    patrol_node = Node(
        package='patrol_navigation_project_cpp',
        executable='patrol_controller',
        name='patrol_controller',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    return LaunchDescription([
        map_arg,
        params_file_arg,
        nav2_launch,
        rviz_node,
        patrol_node
    ])