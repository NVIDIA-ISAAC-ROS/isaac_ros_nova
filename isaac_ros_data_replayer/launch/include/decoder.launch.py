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
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    target_container = LaunchConfiguration('target_container')
    target_container_launch_arg = DeclareLaunchArgument(
        'target_container',
        description='Target Container',
        default_value='container',
    )

    namespace = LaunchConfiguration('namespace')
    namespace_launch_arg = DeclareLaunchArgument(
        'namespace',
        description='Namespace',
        default_value='/',
    )

    launch_args = [
        target_container_launch_arg,
        namespace_launch_arg,
    ]

    decoder_node = ComposableNode(
        name='decoder',
        package='isaac_ros_h264_decoder',
        plugin='nvidia::isaac_ros::h264_decoder::DecoderNode',
        namespace=namespace,
        remappings=[
            ('image_uncompressed', 'image_raw'),
        ],
    )

    container = Node(
        name=target_container,
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        on_exit=Shutdown(),
        condition=LaunchConfigurationEquals('target_container', 'container'),
    )

    load_nodes = LoadComposableNodes(
        target_container=target_container,
        composable_node_descriptions=[decoder_node],
    )

    return LaunchDescription(launch_args + [container, load_nodes])
