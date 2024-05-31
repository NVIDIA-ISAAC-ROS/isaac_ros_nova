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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument('target_container'),
        DeclareLaunchArgument('namespace'),
        DeclareLaunchArgument('camera_id'),
    ]

    target_container = LaunchConfiguration('target_container')
    namespace = LaunchConfiguration('namespace')
    camera_id = LaunchConfiguration('camera_id')

    isaac_ros_owl_launch_dir = os.path.join(
        get_package_share_directory('isaac_ros_owl'), 'launch')

    owl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([isaac_ros_owl_launch_dir, '/owl.launch.py']),
        launch_arguments={
            'target_container': target_container,
            'namespace': namespace,
            'camera_id': camera_id,
        }.items(),
    )

    encoder_node = ComposableNode(
        name='encoder',
        package='isaac_ros_h264_encoder',
        plugin='nvidia::isaac_ros::h264_encoder::EncoderNode',
        namespace=namespace,
        remappings=[
            ('image_raw', 'left/image_raw'),
            ('image_compressed', 'left/image_compressed'),
        ],
        parameters=[{
            'config': 'iframe_cqp',
            'type_negotiation_duration_s': 5,
        }],
    )

    load_nodes = LoadComposableNodes(
        target_container=target_container,
        composable_node_descriptions=[
            encoder_node,
        ],
    )

    return LaunchDescription(launch_args + [owl_launch, load_nodes])
