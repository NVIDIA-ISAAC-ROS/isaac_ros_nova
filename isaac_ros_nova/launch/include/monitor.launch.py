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

from launch import LaunchDescription
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description() -> LaunchDescription:

    container = Node(
        name='nova_container',
        package='rclcpp_components',
        executable='component_container',
        output='both',
    )

    monitor_node = ComposableNode(
        name='monitor',
        package='isaac_ros_nova',
        plugin='nvidia::isaac_ros::nova_monitor::MonitorNode',
    )

    load_nodes = LoadComposableNodes(
        target_container='nova_container',
        composable_node_descriptions=[monitor_node],
    )

    return LaunchDescription([container, load_nodes])
