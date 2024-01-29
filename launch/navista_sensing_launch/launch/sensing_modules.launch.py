# Copyright 2023, 2024 Yuma Matsumura All rights reserved.
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
    sensing_params_path = PathJoinSubstitution(
        [FindPackageShare('navista_sensing_launch'), 'params', 'sensing_modules_params.yaml']
    )

    # Set launch params
    container_name = LaunchConfiguration('container_name')
    sensing_params_file = LaunchConfiguration('sensing_params_file')
    sensing_log_level = LaunchConfiguration('sensing_log_level')
    use_composition = LaunchConfiguration('use_composition')
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name',
        default_value='navista_container',
        description='The name of conatiner that nodes will load in if use composition',
    )
    declare_sensing_params_file_cmd = DeclareLaunchArgument(
        'sensing_params_file',
        default_value=sensing_params_path,
        description='Full path to the ROS 2 parameters file for sensing modules',
    )
    declare_sensing_log_level_cmd = DeclareLaunchArgument(
        'sensing_log_level',
        default_value='info',
        description='Log level for sensing module [DEBUG|INFO|WARN|ERROR|FATAL]',
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
                        name='points_converter_node',
                        package='navista_points_converter',
                        plugin='navista_points_converter::PointsConverter',
                        parameters=[
                            {'rclcpp.logging.min_severity': sensing_log_level},
                            {'use_sim_time': use_sim_time},
                            sensing_params_file,
                        ],
                        remappings=[
                            ('/pcd', '/points'),
                            ('/octomap', '/octo_points'),
                        ],
                    ),
                    ComposableNode(
                        name='voxel_grid_filter_node',
                        package='navista_voxel_grid_filter',
                        plugin='navista_voxel_grid_filter::VoxelGridFilter',
                        parameters=[
                            {'rclcpp.logging.min_severity': sensing_log_level},
                            {'use_sim_time': use_sim_time},
                            sensing_params_file,
                        ],
                        remappings=[
                            ('/input_pcd', '/points'),
                            ('/output_pcd', '/filtered_points'),
                        ],
                    ),
                ],
            )
        ],
    )

    load_nodes = GroupAction(
        condition=UnlessCondition(use_composition),
        actions=[
            Node(
                name='points_converter_node',
                package='navista_points_converter',
                executable='points_converter',
                parameters=[{'use_sim_time': use_sim_time}, sensing_params_file],
                remappings=[
                    ('/pcd', '/points'),
                    ('/octomap', '/octo_points'),
                ],
                arguments=['--ros-args', '--log-level', sensing_log_level],
                output='screen',
            ),
            Node(
                name='voxel_grid_filter_node',
                package='navista_voxel_grid_filter',
                executable='voxel_grid_filter',
                parameters=[{'use_sim_time': use_sim_time}, sensing_params_file],
                remappings=[
                    ('/input_pcd', '/points'),
                    ('/output_pcd', '/filtered_points'),
                ],
                arguments=['--ros-args', '--log-level', sensing_log_level],
                output='screen',
            ),
        ],
    )

    return LaunchDescription(
        [
            declare_container_name_cmd,
            declare_sensing_params_file_cmd,
            declare_sensing_log_level_cmd,
            declare_use_composition_cmd,
            declare_use_sim_time_cmd,
            load_composition_nodes,
            load_nodes,
        ]
    )
