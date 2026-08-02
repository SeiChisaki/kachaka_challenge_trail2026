# Copyright (C) 2023 Kachaka

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Get directories
    sim_dir = get_package_share_directory('kachaka_gazebo')
    nav2_dir = get_package_share_directory('kachaka_nav2_bringup')
    
    # Create launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    headless = LaunchConfiguration('headless', default='True')
    namespace = LaunchConfiguration('namespace', default='')
    map_name = LaunchConfiguration('map_name', default='sample_world')
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation clock if true'
    )
    
    declare_headless_cmd = DeclareLaunchArgument(
        'headless',
        default_value='True',
        description='Run Gazebo in headless mode'
    )
    
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace'
    )

    declare_map_name_cmd = DeclareLaunchArgument(
        'map_name',
        default_value='sample_world',
        description='Map name to load'
    )
    
    # Function to resolve paths with the map name
    def get_paths_and_launch(context):
        name = context.perform_substitution(map_name)
        world_file = os.path.join(sim_dir, 'worlds', f'{name}.sdf')
        map_file = os.path.join(nav2_dir, 'maps', f'{name}.yaml')
        
        # Include simulation launch file
        simulation_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sim_dir, 'launch', 'simulation.launch.py')
            ),
            launch_arguments={
                'headless': headless,
                'use_sim_time': use_sim_time,
                'namespace': namespace,
                'world': world_file
            }.items()
        )
        
        # Include localization launch file
        localization_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_dir, 'launch', 'localization_launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'namespace': namespace,
                'map': map_file
            }.items()
        )

        # Include navigation launch file
        navigation_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'namespace': namespace,
                'autostart': 'true',
                'params_file': os.path.join(nav2_dir, 'params', 'nav2_params.yaml'),
            }.items()
        )

        return [simulation_launch, localization_launch, navigation_launch]

    # Create and return launch description
    ld = LaunchDescription()

    # Add declared arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_headless_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_map_name_cmd)
    
    # Add the actions to launch simulation and localization using OpaqueFunction
    ld.add_action(OpaqueFunction(function=get_paths_and_launch))
    
    return ld