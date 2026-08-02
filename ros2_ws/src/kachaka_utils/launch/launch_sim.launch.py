import os
import subprocess

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _kill_gazebo():
    """起動前に残留している Gazebo プロセスを終了する。"""
    subprocess.run(['pkill', '-f', 'gz_sim'], capture_output=True)
    subprocess.run(['pkill', '-f', 'gz sim'], capture_output=True)


def generate_launch_description():
    sim_dir = get_package_share_directory('kachaka_gazebo')
    nav2_dir = get_package_share_directory('kachaka_nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    headless = LaunchConfiguration('headless', default='False')
    namespace = LaunchConfiguration('namespace', default='')
    map_name = LaunchConfiguration('map_name', default='sample_world')
    task = LaunchConfiguration('task', default='0')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Use simulation clock if true',
    )
    declare_headless_cmd = DeclareLaunchArgument(
        'headless', default_value='False',
        description='Run Gazebo in headless mode',
    )
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='',
        description='Top-level namespace',
    )
    declare_map_name_cmd = DeclareLaunchArgument(
        'map_name', default_value='sample_world',
        description='Map name used in free mode (task:=0)',
    )
    declare_task_cmd = DeclareLaunchArgument(
        'task', default_value='0',
        description=(
            'Task number: 0=free (uses map_name), '
            '1=Task1 ウェイポイントナビゲーション, '
            '2=Task2 ゴミ検出, '
            '3=Task3 完全探索・分類'
        ),
    )

    def get_paths_and_launch(context):
        _kill_gazebo()
        task_str = context.perform_substitution(task)

        if task_str in ('1', '2', '3'):
            world_file = os.path.join(sim_dir, 'worlds', f'task{task_str}_warehouse.sdf')
            map_file = os.path.join(nav2_dir, 'maps', 'warehouse.yaml')
        else:
            name = context.perform_substitution(map_name)
            world_file = os.path.join(sim_dir, 'worlds', f'{name}.sdf')
            map_file = os.path.join(nav2_dir, 'maps', f'{name}.yaml')

        simulation_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sim_dir, 'launch', 'simulation.launch.py')
            ),
            launch_arguments={
                'headless': headless,
                'use_sim_time': use_sim_time,
                'namespace': namespace,
                'world': world_file,
            }.items(),
        )

        localization_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_dir, 'launch', 'localization_launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'namespace': namespace,
                'map': map_file,
            }.items(),
        )

        navigation_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'namespace': namespace,
                'autostart': 'true',
                'params_file': os.path.join(nav2_dir, 'params', 'nav2_params.yaml'),
            }.items(),
        )

        actions = [simulation_launch, localization_launch, navigation_launch]

        if task_str in ('1', '2', '3'):
            judge_node = Node(
                package='trail_task_judge',
                executable=f'task{task_str}_judge_node.py',
                name=f'task{task_str}_judge',
                output='screen',
            )
            actions.append(judge_node)

        return actions

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_headless_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_map_name_cmd)
    ld.add_action(declare_task_cmd)
    ld.add_action(OpaqueFunction(function=get_paths_and_launch))
    return ld
