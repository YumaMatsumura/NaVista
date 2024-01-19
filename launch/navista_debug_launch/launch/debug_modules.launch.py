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
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import AndSubstitution
from launch.substitutions import LaunchConfiguration
from launch.substitutions import NotSubstitution
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get the file directory
    debug_params_path = PathJoinSubstitution(
        [FindPackageShare('navista_debug_launch'), 'params', 'debug_modules_params.yaml']
    )

    # Set launch params
    container_name = LaunchConfiguration('container_name')
    debug_params_file = LaunchConfiguration('debug_params_file')
    use_composition = LaunchConfiguration('use_composition')
    use_debug = LaunchConfiguration('use_debug')
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name',
        default_value='navista_container',
        description='The name of conatiner that nodes will load in if use composition',
    )
    declare_debug_params_file_cmd = DeclareLaunchArgument(
        'debug_params_file',
        default_value=debug_params_path,
        description='Full path to the ROS 2 parameters file for debug modules',
    )
    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='True', description='Whether to use composed nodes'
    )
    declare_use_debug_cmd = DeclareLaunchArgument(
        'use_debug', default_value='False', description='Whether to use debug'
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation clock if true'
    )

    # Load nodes
    load_composition_nodes = GroupAction(
        condition=IfCondition(AndSubstitution(use_composition, use_debug)),
        actions=[
            LoadComposableNodes(
                target_container=container_name,
                composable_node_descriptions=[
                    ComposableNode(
                        name='octomap_debug_node',
                        package='navista_octomap_debug',
                        plugin='navista_octomap_debug::OctomapDebug',
                        parameters=[
                            {'use_sim_time': use_sim_time},
                            debug_params_file,
                        ],
                    ),
                ],
            )
        ],
    )

    load_nodes = GroupAction(
        condition=IfCondition(AndSubstitution(NotSubstitution(use_composition), use_debug)),
        actions=[
            Node(
                name='octomap_debug_node',
                package='navista_octomap_debug',
                executable='octomap_debug',
                parameters=[{'use_sim_time': use_sim_time}, debug_params_file],
                output='screen',
            ),
        ],
    )

    return LaunchDescription(
        [
            declare_container_name_cmd,
            declare_debug_params_file_cmd,
            declare_use_composition_cmd,
            declare_use_debug_cmd,
            declare_use_sim_time_cmd,
            load_composition_nodes,
            load_nodes,
        ]
    )
