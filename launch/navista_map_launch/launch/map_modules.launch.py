# Copyright 2023 Yuma Matsumura All rights reserved.
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
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get the file directory
    map_params_path = PathJoinSubstitution(
        [FindPackageShare('navista_map_launch'), 'params', 'map_modules_params.yaml']
    )

    # Set launch params
    container_name = LaunchConfiguration('container_name')
    map_params_file = LaunchConfiguration('map_params_file')
    use_composition = LaunchConfiguration('use_composition')
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name',
        default_value='navista_container',
        description='The name of conatiner that nodes will load in if use composition',
    )
    declare_map_params_file_cmd = DeclareLaunchArgument(
        'map_params_file',
        default_value=map_params_path,
        description='Full path to the ROS 2 parameters file for map modules',
    )
    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='True', description='Whether to use composed nodes'
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation clock if true'
    )

    # Load nodes
    load_composition_nodes = GroupAction(
        condition=IfCondition(use_composition),
        actions=[
            LoadComposableNodes(
                target_container=container_name,
                composable_node_descriptions=[
                    ComposableNode(
                        name='map_loader_node',
                        package='navista_map_loader',
                        plugin='navista_map_loader::MapLoader',
                        parameters=[{'use_sim_time': use_sim_time}, map_params_file],
                    ),
                ],
            )
        ],
    )

    load_nodes = GroupAction(
        condition=UnlessCondition(use_composition),
        actions=[
            Node(
                name='map_loader_node',
                package='navista_map_loader',
                executable='map_loader',
                parameters=[{'use_sim_time': use_sim_time}, map_params_file],
                output='screen',
            ),
        ],
    )

    return LaunchDescription(
        [
            declare_container_name_cmd,
            declare_map_params_file_cmd,
            declare_use_composition_cmd,
            declare_use_sim_time_cmd,
            load_composition_nodes,
            load_nodes,
        ]
    )
