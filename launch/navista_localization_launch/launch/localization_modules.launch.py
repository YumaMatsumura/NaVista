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
from launch.actions import EmitEvent
from launch.actions import GroupAction
from launch.actions import LogInfo
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    # Get the file directory
    localization_params_path = PathJoinSubstitution(
        [
            FindPackageShare('navista_localization_launch'),
            'params',
            'localization_modules_params.yaml',
        ]
    )

    # Set launch params
    localization_params_file = LaunchConfiguration('localization_params_file')
    localization_log_level = LaunchConfiguration('localization_log_level')
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_localization_params_file_cmd = DeclareLaunchArgument(
        'localization_params_file',
        default_value=localization_params_path,
        description='Full path to the ROS 2 parameters file for map modules',
    )
    declare_localization_log_level_cmd = DeclareLaunchArgument(
        'localization_log_level',
        default_value='info',
        description='Log level for localization module [DEBUG|INFO|WARN|ERROR|FATAL]',
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true',
    )

    # Load nodes
    lidar_localization = LifecycleNode(
        name='lidar_localization',
        namespace='',
        package='lidar_localization_ros2',
        executable='lidar_localization_node',
        parameters=[
            {'use_sim_time': use_sim_time},
            localization_params_file,
        ],
        remappings=[
            ('pcl_pose', '/current_pose'),
            ('path', '/pose_path'),
            ('map', '/pcd_map'),
            ('odom', '/odom'),
            ('velodyne_points', '/filtered_points'),
            ('imu', '/imu/data'),
        ],
        arguments=['--ros-args', '--log-level', localization_log_level],
    )

    to_inactive = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(lidar_localization),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    from_unconfigured_to_inactive = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=lidar_localization,
            goal_state='unconfigured',
            entities=[
                LogInfo(msg="-- Unconfigured --"),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(lidar_localization),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    )
                ),
            ],
        )
    )

    from_inactive_to_active = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=lidar_localization,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                LogInfo(msg="-- Inactive --"),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(lidar_localization),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        )
    )

    return LaunchDescription(
        [
            declare_localization_params_file_cmd,
            declare_localization_log_level_cmd,
            declare_use_sim_time_cmd,
            lidar_localization,
            to_inactive,
            from_unconfigured_to_inactive,
            from_inactive_to_active,
        ]
    )
