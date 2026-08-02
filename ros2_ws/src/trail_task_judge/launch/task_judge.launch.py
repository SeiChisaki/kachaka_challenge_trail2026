import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    task = LaunchConfiguration('task', default='1')

    declare_task_cmd = DeclareLaunchArgument(
        'task',
        default_value='1',
        description='Task number (1, 2, or 3)',
    )

    def get_judge_node(context):
        task_num = context.perform_substitution(task)
        script_map = {
            '1': 'task1_judge_node.py',
            '2': 'task2_judge_node.py',
            '3': 'task3_judge_node.py',
        }
        script = script_map.get(task_num)
        if script is None:
            return []
        return [
            Node(
                package='trail_task_judge',
                executable=script,
                name=f'task{task_num}_judge',
                output='screen',
            )
        ]

    ld = LaunchDescription()
    ld.add_action(declare_task_cmd)
    ld.add_action(OpaqueFunction(function=get_judge_node))
    return ld
