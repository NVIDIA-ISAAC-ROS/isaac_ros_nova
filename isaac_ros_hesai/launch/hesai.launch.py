# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

import os
import sys
from tempfile import NamedTemporaryFile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.actions import OpaqueFunction, Shutdown, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def load_config(context: LaunchContext,
                namespace: LaunchConfiguration,
                replay: LaunchConfiguration):
    config_path = os.path.join(
        get_package_share_directory('hesai_ros_driver'), 'config', 'config.yaml')
    correction_path = os.path.join(
        get_package_share_directory('isaac_ros_hesai'), 'config', 'correction.csv')
    firetimes_path = os.path.join(
        get_package_share_directory('isaac_ros_hesai'), 'config', 'firetimes.csv')

    config = None
    with open(config_path, 'r') as file:
        config = yaml.safe_load(file)
        config['lidar'][0]['driver']['correction_file_path'] = correction_path
        config['lidar'][0]['driver']['firetimes_path'] = firetimes_path
        if context.perform_substitution(replay) == 'True':
            config['lidar'][0]['driver']['source_type'] = 3
        else:
            config['lidar'][0]['driver']['source_type'] = 1

    if config is None:
        print('Error loading config from ' + config_path)
        sys.exit(1)

    tmp = NamedTemporaryFile(suffix='.yaml', delete=False)
    with open(tmp.name, 'w') as file:
        yaml.dump(config, file)

    hesai_lidar_node = Node(
        name='hesai_lidar',
        package='hesai_ros_driver',
        executable='hesai_ros_driver_node',
        arguments=['-c', tmp.name],
        namespace=namespace,
        remappings=[
            ('/lidar_packets', 'lidar_packets'),
            ('/lidar_points', 'lidar_points'),
        ],
        output='screen',
        on_exit=Shutdown(),
    )

    return [LaunchDescription([
        hesai_lidar_node,
        TimerAction(actions=[ExecuteProcess(cmd=['rm', '-f', 'log.log'])], period=1.0),
    ])]


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    namespace_launch_arg = DeclareLaunchArgument(
        'namespace',
        description='Namespace',
        default_value='hesai_lidar',
    )

    replay = LaunchConfiguration('replay')
    replay_launch_arg = DeclareLaunchArgument(
        'replay',
        description='Enable replay from rosbag',
        default_value='False',
    )

    launch_args = [
        namespace_launch_arg,
        replay_launch_arg,
    ]

    return LaunchDescription(launch_args + [
        OpaqueFunction(function=load_config, args=[namespace, replay]),
    ])
