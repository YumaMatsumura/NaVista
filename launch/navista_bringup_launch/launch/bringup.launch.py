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
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get the file directory
    map_launch_path = PathJoinSubstitution(
        [FindPackageShare('navista_map_launch'), 'launch', 'map_modules.launch.py']
    )
    map_params_path = PathJoinSubstitution(
        [FindPackageShare('navista_map_launch'), 'params', 'map_modules_params.yaml']
    )
    sensing_launch_path = PathJoinSubstitution(
        [FindPackageShare('navista_sensing_launch'), 'launch', 'sensing_modules.launch.py']
    )
    sensing_params_path = PathJoinSubstitution(
        [FindPackageShare('navista_sensing_launch'), 'params', 'sensing_modules_params.yaml']
    )
    # rviz_path = PathJoinSubstitution(
    #    [FindPackageShare('mapvista_bringup_launch'), 'rviz', 'bringup.rviz']
    # )

    # Set launch params
    container_name = LaunchConfiguration('container_name')
    map_yaml_file = LaunchConfiguration('map_yaml_file')
    map_launch_file = LaunchConfiguration('map_launch_file')
    map_params_file = LaunchConfiguration('map_params_file')
    sensing_launch_file = LaunchConfiguration('sensing_launch_file')
    sensing_params_file = LaunchConfiguration('sensing_params_file')
    use_composition = LaunchConfiguration('use_composition')
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name',
        default_value='navista_container',
        description='The name of conatiner that nodes will load in if use composition',
    )
    declare_map_yaml_file_cmd = DeclareLaunchArgument(
        'map_yaml_file', default_value='', description='Full path to the map yaml_file'
    )
    declare_map_launch_file_cmd = DeclareLaunchArgument(
        'map_launch_file',
        default_value=map_launch_path,
        description='Full path to the ROS 2 launch file for map modules',
    )
    declare_map_params_file_cmd = DeclareLaunchArgument(
        'map_params_file',
        default_value=map_params_path,
        description='Full path to the ROS 2 parameters file for map modules',
    )
    declare_sensing_launch_file_cmd = DeclareLaunchArgument(
        'sensing_launch_file',
        default_value=sensing_launch_path,
        description='Full path to the ROS 2 launch file for sensing modules',
    )
    declare_sensing_params_file_cmd = DeclareLaunchArgument(
        'sensing_params_file',
        default_value=sensing_params_path,
        description='Full path to the ROS 2 parameters file for sensing modules',
    )
    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='True', description='Whether to use composed nodes'
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation clock if true'
    )

    # Load nodes
    load_nodes = GroupAction(
        [
            Node(
                condition=IfCondition(use_composition),
                name=container_name,
                package='rclcpp_components',
                executable='component_container',
                output='screen',
            ),
            # Node(
            #    name='rviz2',
            #    package='rviz2',
            #    executable='rviz2',
            #    arguments=['-d', rviz_path],
            #    parameters=[{'use_sim_time': use_sim_time}],
            #    output='screen',
            # ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([map_launch_file]),
                launch_arguments={
                    'container_name': container_name,
                    'map_yaml_file': map_yaml_file,
                    'map_params_file': map_params_file,
                    'use_composition': use_composition,
                    'use_sim_time': use_sim_time,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([sensing_launch_file]),
                launch_arguments={
                    'container_name': container_name,
                    'sensing_params_file': sensing_params_file,
                    'use_composition': use_composition,
                    'use_sim_time': use_sim_time,
                }.items(),
            ),
        ]
    )

    return LaunchDescription(
        [
            declare_container_name_cmd,
            declare_map_yaml_file_cmd,
            declare_map_launch_file_cmd,
            declare_map_params_file_cmd,
            declare_sensing_launch_file_cmd,
            declare_sensing_params_file_cmd,
            declare_use_composition_cmd,
            declare_use_sim_time_cmd,
            load_nodes,
        ]
    )
