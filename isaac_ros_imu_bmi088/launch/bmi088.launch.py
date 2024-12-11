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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):

    target_container = LaunchConfiguration('target_container')
    namespace = LaunchConfiguration('namespace')
    bmi_id = LaunchConfiguration('bmi_id')
    imu_frequency = LaunchConfiguration('imu_frequency')

    parameters_list = [{
            'bmi_id': bmi_id,
            'imu_frequency': imu_frequency,
            'type_negotiation_duration_s': 5,
        }]

    if LaunchConfigurationEquals('diagnostics', 'True').evaluate(context):
        parameters_list[0].update({
            'enable_msg_time_diagnostics': True,
            'enable_increasing_msg_time_diagnostics': True,
            'diagnostics_publish_rate': 1.0,
            'filter_window_size': 30,
            'topics_list': ['imu'],
            'expected_fps_list': [float(imu_frequency.perform(context))],
        })

    bmi088_node = ComposableNode(
        name='bmi088',
        package='isaac_ros_imu_bmi088',
        plugin='nvidia::isaac_ros::imu_bmi088::Bmi088Node',
        namespace=namespace,
        remappings=[
            ('correlated_timestamp', '/correlated_timestamp')
        ],
        parameters=parameters_list,
    )

    load_nodes = LoadComposableNodes(
        target_container=target_container,
        composable_node_descriptions=[bmi088_node],
    )

    return [load_nodes]


def generate_launch_description():
    target_container = LaunchConfiguration('target_container')
    target_container_launch_arg = DeclareLaunchArgument(
        'target_container',
        description='Target Container',
        default_value='bmi088_container',
    )

    namespace_launch_arg = DeclareLaunchArgument(
        'namespace',
        description='Namespace',
        default_value='imu',
    )

    bmi_id_launch_arg = DeclareLaunchArgument(
        'bmi_id',
        description='BMI ID',
        default_value='69',
    )

    imu_frequency_launch_arg = DeclareLaunchArgument(
        'imu_frequency',
        description='IMU Frequency',
        default_value='100',
    )

    diagnostics_launch_arg = DeclareLaunchArgument(
        'diagnostics',
        default_value='False',
        description='Turns on/off publishing to the diagnostics topic.'
    )

    launch_args = [
        target_container_launch_arg,
        namespace_launch_arg,
        bmi_id_launch_arg,
        imu_frequency_launch_arg,
        diagnostics_launch_arg,
    ]

    container = Node(
        name=target_container,
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        on_exit=Shutdown(),
        condition=LaunchConfigurationEquals('target_container', 'bmi088_container'),
    )

    isaac_ros_correlated_timestamp_driver_launch_dir = os.path.join(
        get_package_share_directory('isaac_ros_correlated_timestamp_driver'), 'launch')

    correlated_timestamp_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            isaac_ros_correlated_timestamp_driver_launch_dir,
            '/correlated_timestamp_driver.launch.py',
        ]),
        launch_arguments={'target_container': target_container}.items(),
        condition=LaunchConfigurationEquals('target_container', 'bmi088_container'),
    )

    return LaunchDescription(launch_args + [
        container,
        correlated_timestamp_driver_launch,
        OpaqueFunction(function=launch_setup),
    ])
