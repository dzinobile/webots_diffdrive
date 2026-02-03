#!/usr/bin/env python3
"""
Multi-Robot Nav2 Launch File for Webots Simulation
This launch file starts:
- Webots simulator with multi-robot world
- SLAM Toolbox for each robot (mapping and localization)
- Nav2 stack for each robot (navigation)
- All necessary TF transforms
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
    EmitEvent
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    # Package directories
    package_dir = get_package_share_directory('webots_diffdrive')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart = LaunchConfiguration('autostart', default='true')
    
    # Webots launcher
    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'custom_world.wbt'),
        mode='realtime'
    )
    
    # Lists to hold nodes for each robot
    robot_drivers = []
    slam_nodes = []
    nav2_groups = []
    
    # Number of robots
    num_robots = 2
    
    for i in range(1, num_robots + 1):
        robot_name = f'robot{i}'
        namespace = robot_name
        
        # Robot description path
        robot_description_path = os.path.join(
            package_dir, 'resource', f'{robot_name}.urdf'
        )
        
        # Webots controller for the robot
        robot_driver = WebotsController(
            robot_name=robot_name,
            # namespace=namespace,
            parameters=[
                {'robot_description': robot_description_path},
                {'use_sim_time': use_sim_time}
            ],
            remappings=[
                ('cmd_vel', f'robot{i}/cmd_vel'),
            ],
            
        )
        robot_drivers.append(robot_driver)
        
        # SLAM Toolbox node for mapping and localization
        slam_params_file = os.path.join(
            package_dir, 'config', 'slam_toolbox_params.yaml'
        )
        
        slam_node = Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            namespace=namespace,
            parameters=[
                slam_params_file,
                {
                    'use_sim_time': use_sim_time,
                    'odom_frame': f'{namespace}/odom',
                    'map_frame': f'{namespace}/map',
                    'base_frame': f'{namespace}/base_link',
                    'scan_topic': f'/{namespace}/scan',
                }
            ],
            output='screen',
            remappings=[
                ('scan', f'{namespace}/scan'),
            ]
        )
        slam_nodes.append(slam_node)
        
        # Nav2 parameters file for this robot
        nav2_params_file = os.path.join(
            package_dir, 'config', f'nav2_params_{robot_name}.yaml'
        )
        
        # Nav2 navigation stack
        nav2_group = GroupAction([
            PushRosNamespace(namespace),
            
            # Include Nav2 bringup launch file
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'namespace': namespace,
                    'use_sim_time': 'true',
                    'autostart': autostart,
                    'params_file': nav2_params_file,
                    'use_composition': 'False',
                    'use_respawn': 'False',
                }.items()
            ),
        ])
        nav2_groups.append(nav2_group)
    
    # Shutdown handler - shutdown when Webots exits
    shutdown_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=webots,
            on_exit=[EmitEvent(event=Shutdown())],
        )
    )
    
    # Create and return launch description
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically startup the nav2 stack'
        ),
        
        # Start Webots
        webots,
        
        # Start robot drivers
        *robot_drivers,
        
        # Start SLAM for each robot
        *slam_nodes,
        
        # Start Nav2 for each robot
        *nav2_groups,
        
        # Shutdown handler
        shutdown_handler,
    ])