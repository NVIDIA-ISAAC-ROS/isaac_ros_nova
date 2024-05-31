# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2023-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    name = LaunchConfiguration('name')
    name_launch_arg = DeclareLaunchArgument(
        'name',
        description='Name',
        default_value='sllidar',
    )

    channel_type = LaunchConfiguration('channel_type')
    channel_type_launch_arg = DeclareLaunchArgument(
        'channel_type',
        description='Channel Type',
        default_value='udp',
    )

    udp_ip = LaunchConfiguration('udp_ip')
    udp_ip_launch_arg = DeclareLaunchArgument(
        'udp_ip',
        description='UDP IP',
        default_value='192.168.1.2',
    )

    udp_port = LaunchConfiguration('udp_port')
    udp_port_launch_arg = DeclareLaunchArgument(
        'udp_port',
        description='UDP Port',
        default_value='8089',
    )

    inverted = LaunchConfiguration('inverted')
    inverted_launch_arg = DeclareLaunchArgument(
        'inverted',
        description='Inverted',
        default_value='False',
    )

    angle_compensate = LaunchConfiguration('angle_compensate')
    angle_compensate_launch_arg = DeclareLaunchArgument(
        'angle_compensate',
        description='Angle Compensate',
        default_value='True',
    )

    scan_mode = LaunchConfiguration('scan_mode')
    scan_mode_launch_arg = DeclareLaunchArgument(
        'scan_mode',
        description='Scan Mode',
        default_value='Sensitivity',
    )

    launch_args = [
        name_launch_arg,
        channel_type_launch_arg,
        udp_ip_launch_arg,
        udp_port_launch_arg,
        inverted_launch_arg,
        angle_compensate_launch_arg,
        scan_mode_launch_arg,
    ]

    frame_id = PythonExpression(["'", name, "_driver'"])

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
            'frame_id': frame_id,
        }],
        output='screen',
        on_exit=Shutdown(),
    )

    return LaunchDescription(launch_args + [sllidar_node])
