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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
import yaml


def generate_launch_description():
    systeminfo_file_path = '/etc/nova/systeminfo.yaml'
    first_hawk_module_id = None
    if not os.path.exists(systeminfo_file_path):
        raise FileNotFoundError(f'{systeminfo_file_path} does not exist. This file is required for'
                                ' isaac_ros_hawk, if you are not using a Nova system see the'
                                ' isaac_ros_argus_camera package')

    with open('/etc/nova/systeminfo.yaml', 'r') as systeminfo_file:
        systeminfo = yaml.safe_load(systeminfo_file)
        for _, sensor_info in systeminfo['sensors'].items():
            if sensor_info['type'] == 'hawk' and 'module_id' in sensor_info:
                first_hawk_module_id = str(sensor_info['module_id'])
                break
        if first_hawk_module_id is None:
            raise ValueError('No Hawk sensor with module_id found in systeminfo.yaml')

    target_container = LaunchConfiguration('target_container')
    target_container_launch_arg = DeclareLaunchArgument(
        'target_container',
        description='Target Container',
        default_value='hawk_container',
    )

    namespace = LaunchConfiguration('namespace')
    namespace_launch_arg = DeclareLaunchArgument(
        'namespace',
        description='Namespace',
        default_value='stereo_camera',
    )

    module_id = LaunchConfiguration('module_id')
    module_id_launch_arg = DeclareLaunchArgument(
        'module_id',
        description='Index specifying the camera module to use.',
        default_value=first_hawk_module_id,
    )

    mode = LaunchConfiguration('mode')
    mode_launch_arg = DeclareLaunchArgument(
        'mode',
        description='Supported Resolution mode from the camera. For example, 0: 1920 x 1200',
        default_value='0',
    )

    fsync_type = LaunchConfiguration('fsync_type')
    fsync_type_launch_arg = DeclareLaunchArgument(
        'fsync_type',
        default_value='1',
        description='Specifies what kind of Frame Synchronization to use. Supported values are: '
                    '0 for internal, 1 for external.',
    )

    wide_fov = LaunchConfiguration('wide_fov')
    wide_fov_launch_arg = DeclareLaunchArgument(
        'wide_fov',
        default_value='0',
        description='Specifies FoV mode. Supported values are: 0 normal, 1 for wide.'
    )

    launch_args = [
        target_container_launch_arg,
        namespace_launch_arg,
        module_id_launch_arg,
        mode_launch_arg,
        fsync_type_launch_arg,
        wide_fov_launch_arg,
    ]

    container = Node(
        name=target_container,
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        on_exit=Shutdown(),
        condition=LaunchConfigurationEquals('target_container', 'hawk_container'),
    )

    isaac_ros_correlated_timestamp_driver_launch_dir = os.path.join(
        get_package_share_directory('isaac_ros_correlated_timestamp_driver'), 'launch')

    correlated_timestamp_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            isaac_ros_correlated_timestamp_driver_launch_dir,
            '/correlated_timestamp_driver.launch.py',
        ]),
        launch_arguments={'target_container': target_container}.items(),
        condition=LaunchConfigurationEquals('target_container', 'hawk_container'),
    )

    left_optical_frame_name = PythonExpression(["'", namespace, "_left_optical'"])
    right_optical_frame_name = PythonExpression(["'", namespace, "_right_optical'"])

    hawk_node = ComposableNode(
        name='hawk',
        package='isaac_ros_hawk',
        plugin='nvidia::isaac_ros::hawk::HawkNode',
        namespace=namespace,
        remappings=[
            ('correlated_timestamp', '/correlated_timestamp'),
        ],
        parameters=[{
            'module_id': module_id,
            'mode': mode,
            'fsync_type': fsync_type,
            'wide_fov': wide_fov,
            'camera_link_frame_name': namespace,
            'left_optical_frame_name': left_optical_frame_name,
            'right_optical_frame_name': right_optical_frame_name,
            'type_negotiation_duration_s': 5,
        }],
    )

    load_nodes = LoadComposableNodes(
        target_container=target_container,
        composable_node_descriptions=[hawk_node],
    )

    return LaunchDescription(launch_args + [
        container,
        correlated_timestamp_driver_launch,
        load_nodes,
    ])
