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

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    OpaqueFunction, Shutdown
)
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):
    target_container = LaunchConfiguration('target_container')
    namespace = LaunchConfiguration('namespace')
    camera_id = LaunchConfiguration('camera_id')
    mode = LaunchConfiguration('mode')
    fsync_type = LaunchConfiguration('fsync_type')

    optical_frame_name = PythonExpression(["'", namespace, "_optical'"])

    parameters_list = [{
            'camera_id': camera_id,
            'mode': mode,
            'fsync_type': fsync_type,
            'camera_link_frame_name': namespace,
            'optical_frame_name': optical_frame_name,
            'type_negotiation_duration_s': 5,
        }]

    if LaunchConfigurationEquals('diagnostics', 'True').evaluate(context):
        parameters_list[0].update({
            'enable_msg_time_diagnostics': True,
            'enable_increasing_msg_time_diagnostics': True,
            'diagnostics_publish_rate': 1.0,
            'filter_window_size': 30,
            'topics_list': ['left/image_raw', 'left/camera_info',
                            'right/image_raw', 'right/camera_info'],
            'expected_fps_list': [30.0, 30.0, 30.0, 30.0],
        })

    owl_node = ComposableNode(
        name='owl',
        package='isaac_ros_owl',
        plugin='nvidia::isaac_ros::owl::OwlNode',
        namespace=namespace,
        remappings=[
            ('correlated_timestamp', '/correlated_timestamp'),
        ],
        parameters=parameters_list,
    )

    load_nodes = LoadComposableNodes(
        target_container=target_container,
        composable_node_descriptions=[owl_node],
    )

    return [load_nodes]


def generate_launch_description():
    target_container = LaunchConfiguration('target_container')
    target_container_launch_arg = DeclareLaunchArgument(
        'target_container',
        description='Target Container',
        default_value='owl_container',
    )

    namespace_launch_arg = DeclareLaunchArgument(
        'namespace',
        description='Namespace',
        default_value='fisheye_camera',
    )

    camera_id_launch_arg = DeclareLaunchArgument(
        'camera_id',
        description='System device numeral for the camera. For example select 0 for /dev/video0.',
        default_value='0',
    )

    mode_launch_arg = DeclareLaunchArgument(
        'mode',
        description='Supported Resolution mode from the camera. For example, 0: 1920 x 1200',
        default_value='0',
    )

    fsync_type_launch_arg = DeclareLaunchArgument(
        'fsync_type',
        default_value='1',
        description='Specifies what kind of Frame Synchronization to use. Supported values are: '
                    '0 for internal, 1 for external.',
    )

    diagnostics_launch_arg = DeclareLaunchArgument(
        'diagnostics',
        default_value='False',
        description='Turns on/off publishing to the diagnostics topic.'
    )

    launch_args = [
        target_container_launch_arg,
        namespace_launch_arg,
        camera_id_launch_arg,
        mode_launch_arg,
        fsync_type_launch_arg,
        diagnostics_launch_arg,
    ]

    container = Node(
        name=target_container,
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        on_exit=Shutdown(),
        condition=LaunchConfigurationEquals('target_container', 'owl_container'),
    )

    isaac_ros_correlated_timestamp_driver_launch_dir = os.path.join(
        get_package_share_directory('isaac_ros_correlated_timestamp_driver'), 'launch')

    correlated_timestamp_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            isaac_ros_correlated_timestamp_driver_launch_dir,
            '/correlated_timestamp_driver.launch.py',
        ]),
        launch_arguments={'target_container': target_container}.items(),
        condition=LaunchConfigurationEquals('target_container', 'owl_container'),
    )

    return LaunchDescription(launch_args + [
        container,
        correlated_timestamp_driver_launch,
        OpaqueFunction(function=launch_setup),
    ])
