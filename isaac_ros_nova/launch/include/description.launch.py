
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
from pathlib import Path

import isaac_ros_launch_utils as lu
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    actions = []

    # Launch robot description depending on nova robot type
    # Uses nominals on system if platform type is not supported
    nova_platform = lu.get_nova_robot()
    if nova_platform is lu.NovaRobot.NOVA_CARTER:
        actions.append(
            lu.include('nova_carter_description', 'launch/nova_carter_description.launch.py'))
    elif nova_platform is lu.NovaRobot.NOVA_DEVELOPER_KIT:
        actions.append(
            lu.include('nova_developer_kit_description',
                       'launch/nova_developer_kit_description.launch.py'))
    else:
        # Use generated calibration file if it exists or use the nominals
        urdf_calibrated = Path('/etc/nova/calibration/isaac_calibration.urdf')
        urdf_nominals = Path('/etc/nova/calibration/isaac_nominals.urdf')
        urdf_filepath = urdf_calibrated
        if not urdf_filepath.exists():
            urdf_filepath = urdf_nominals
        if urdf_filepath.exists():
            actions.append(
                lu.log_info(
                    f'Using URDF: {urdf_filepath}'
                )
            )
            robot_desc = urdf_filepath.read_text()
            robot_state_publisher_node = Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{
                    'robot_description': robot_desc
                }],
            )
            actions.append(robot_state_publisher_node)
        else:
            actions.append(
                lu.log_info(
                    f'No calibration files found on system. Checked calibrated location '
                    f'"{urdf_calibrated}" and nominals location "{urdf_nominals}".'
                    f' Launching without robot description.'
                )
            )

    return LaunchDescription(actions)
