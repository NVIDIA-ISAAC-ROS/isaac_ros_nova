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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    container = Node(
        name='data_recorder',
        package='rclcpp_components',
        executable='component_container_mt',
        on_exit=Shutdown(),
        output='both'
    )

    isaac_ros_data_recorder_launch_include_dir = os.path.join(
        get_package_share_directory('isaac_ros_data_recorder'), 'launch', 'include')

    recorder_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            isaac_ros_data_recorder_launch_include_dir,
            '/recorder.launch.py'
        ])
    )

    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            isaac_ros_data_recorder_launch_include_dir,
            '/sensors.launch.py'
        ]),
        launch_arguments={
            'target_container': 'data_recorder',
        }.items(),
    )

    return LaunchDescription([container, recorder_launch, sensors_launch])
