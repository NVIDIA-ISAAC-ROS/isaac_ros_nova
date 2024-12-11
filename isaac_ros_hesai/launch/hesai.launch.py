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
from typing import List

from ament_index_python.packages import get_package_share_directory
import isaac_ros_launch_utils as lu
from launch import Action, LaunchDescription
from launch.actions import ExecuteProcess, Shutdown, TimerAction
from launch_ros.actions import Node
import yaml


def load_config(args: lu.ArgumentContainer) -> List[Action]:
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
        if lu.is_true(args.replay):
            config['lidar'][0]['driver']['source_type'] = 3
        else:
            config['lidar'][0]['driver']['source_type'] = 1
        config['lidar'][0]['driver']['device_ip_address'] = args.ip
        config['lidar'][0]['ros']['ros_frame_id'] = args.namespace

    if config is None:
        print('Error loading config from ' + config_path)
        sys.exit(1)

    tmp = NamedTemporaryFile(suffix='.yaml', delete=False)
    with open(tmp.name, 'w') as file:
        yaml.dump(config, file)

    actions = []
    actions.append(
        lu.log_info(["Enabling hesai lidar at IP '", args.ip, "'"]))
    actions.append(
        lu.log_info(['lidar source type {}'.format(config['lidar'][0]['driver']['source_type'])]))

    actions.append(Node(
        name='hesai_lidar',
        package='hesai_ros_driver',
        executable='hesai_ros_driver_node',
        arguments=['-c', tmp.name],
        namespace=args.namespace,
        remappings=[
            ('/lidar_packets', 'lidar_packets'),
            ('/lidar_points', 'lidar_points'),
        ],
        output='screen',
        on_exit=Shutdown(),
    ))
    actions.append(TimerAction(actions=[ExecuteProcess(cmd=['rm', '-f', 'log.log'])], period=1.0))
    return actions


def generate_launch_description():
    args = lu.ArgumentContainer()
    args.add_arg('namespace', 'hesai_lidar', 'ROS namespace', cli=True)
    args.add_arg('replay', 'False', 'Enable replay from rosbag', cli=True)
    args.add_arg('ip', '192.168.1.201', 'Hesai source IP address', cli=True)

    args.add_opaque_function(load_config)
    actions = args.get_launch_actions()

    return LaunchDescription(actions)
