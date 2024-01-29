# Copyright 2024 Yuma Matsumura All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the file directory
    # system_params_path = PathJoinSubstitution(
    #    [FindPackageShare('navista_system_launch'), 'params', 'system_modules_params.yaml']
    # )

    # Set launch params
    # system_params_file = LaunchConfiguration('system_params_file')
    system_log_level = LaunchConfiguration('system_log_level')
    use_sim_time = LaunchConfiguration('use_sim_time')
    # declare_system_params_file_cmd = DeclareLaunchArgument(
    #    'system_params_file',
    #    default_value=system_params_path,
    #    description='Full path to the ROS 2 parameters file for system modules',
    # )
    declare_system_log_level_cmd = DeclareLaunchArgument(
        'system_log_level',
        default_value='info',
        description='Log level for system module [DEBUG|INFO|WARN|ERROR|FATAL]',
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation clock if true'
    )

    # Load nodes
    load_nodes = GroupAction(
        [
            Node(
                name='cpu_monitor_node',
                package='navista_cpu_monitor',
                executable='cpu_monitor',
                parameters=[{'use_sim_time': use_sim_time}],  # , system_params_file],
                arguments=['--ros-args', '--log-level', system_log_level],
                output='screen',
            ),
            Node(
                name='memory_monitor_node',
                package='navista_memory_monitor',
                executable='memory_monitor',
                parameters=[{'use_sim_time': use_sim_time}],  # , system_params_file],
                arguments=['--ros-args', '--log-level', system_log_level],
                output='screen',
            ),
        ],
    )

    return LaunchDescription(
        [
            # declare_system_params_file_cmd,
            declare_system_log_level_cmd,
            declare_use_sim_time_cmd,
            load_nodes,
        ]
    )
