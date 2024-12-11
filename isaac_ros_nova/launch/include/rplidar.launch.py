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

import isaac_ros_launch_utils as lu
from launch import LaunchDescription
from launch.actions import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    args = lu.ArgumentContainer()
    name = args.add_arg('name', 'sllidar', 'Name', cli=True)
    channel_type = args.add_arg('channel_type', 'udp', 'Channel Type', cli=True)
    udp_ip = args.add_arg('udp_ip', '192.168.1.2', 'UDP IP', cli=True)
    udp_port = args.add_arg('udp_port', '8089', 'UDP Port', cli=True)
    inverted = args.add_arg('inverted', 'False', 'Inverted', cli=True)
    angle_compensate = args.add_arg('angle_compensate', 'True', 'Angle Compensate', cli=True)
    scan_mode = args.add_arg('scan_mode', 'Sensitivity', 'Scan Mode', cli=True)

    sllidar_node = Node(
        name='sllidar',
        package='sllidar_ros2',
        executable='sllidar_node',
        namespace=name,
        parameters=[{
            'channel_type': channel_type,
            'udp_ip': udp_ip,
            'udp_port': udp_port,
            'inverted': inverted,
            'angle_compensate': angle_compensate,
            'scan_mode': scan_mode,
            'frame_id': name,
        }],
        output='screen',
        on_exit=Shutdown(),
    )

    actions = args.get_launch_actions()
    actions.append(sllidar_node)

    return LaunchDescription(actions)
