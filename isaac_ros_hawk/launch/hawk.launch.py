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
import yaml


def launch_setup(context, *args, **kwargs):

    target_container = LaunchConfiguration('target_container')
    namespace = LaunchConfiguration('namespace')
    module_id = LaunchConfiguration('module_id')
    mode = LaunchConfiguration('mode')
    fsync_type = LaunchConfiguration('fsync_type')
    wide_fov = LaunchConfiguration('wide_fov')
    left_camera_info_url = LaunchConfiguration('left_camera_info_url')
    right_camera_info_url = LaunchConfiguration('right_camera_info_url')

    left_camera_frame_name = PythonExpression(["'", namespace, "_left'"])
    right_camera_frame_name = PythonExpression(["'", namespace, "_right'"])

    parameters_list = [{
        'module_id': module_id,
        'mode': mode,
        'fsync_type': fsync_type,
        'wide_fov': wide_fov,
        'camera_link_frame_name': namespace,
        'left_camera_frame_name': left_camera_frame_name,
        'right_camera_frame_name': right_camera_frame_name,
        'left_camera_info_url': left_camera_info_url,
        'right_camera_info_url': right_camera_info_url,
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

    hawk_node = ComposableNode(
        name='hawk',
        package='isaac_ros_hawk',
        plugin='nvidia::isaac_ros::hawk::HawkNode',
        namespace=namespace,
        remappings=[
            ('correlated_timestamp', '/correlated_timestamp'),
        ],
        parameters=parameters_list,
    )

    load_nodes = LoadComposableNodes(
        target_container=target_container,
        composable_node_descriptions=[hawk_node],
    )

    return [load_nodes]


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

    namespace_launch_arg = DeclareLaunchArgument(
        'namespace',
        description='Namespace',
        default_value='stereo_camera',
    )

    module_id_launch_arg = DeclareLaunchArgument(
        'module_id',
        description='Index specifying the camera module to use,'
        'to use the front camera use index 5 on Nova Carter and 3 on Nova Devkit'
        'for other systems check the /etc/nova/systeminfo.yaml file',
        default_value=first_hawk_module_id,
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

    wide_fov_launch_arg = DeclareLaunchArgument(
        'wide_fov',
        default_value='False',
        description='Specifies FoV mode. Supported values are: False normal, True for wide.'
    )

    left_camera_info_url = DeclareLaunchArgument(
        'left_camera_info_url',
        default_value='',
        description='URL for the left camera info file.'
    )

    right_camera_info_url = DeclareLaunchArgument(
        'right_camera_info_url',
        default_value='',
        description='URL for the right camera info file.'
    )

    diagnostics_launch_arg = DeclareLaunchArgument(
        'diagnostics',
        default_value='False',
        description='Turns on/off publishing to the diagnostics topic.'
    )

    launch_args = [
        target_container_launch_arg,
        namespace_launch_arg,
        module_id_launch_arg,
        mode_launch_arg,
        fsync_type_launch_arg,
        wide_fov_launch_arg,
        diagnostics_launch_arg,
        left_camera_info_url,
        right_camera_info_url,
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

    return LaunchDescription(launch_args + [
        container,
        correlated_timestamp_driver_launch,
        OpaqueFunction(function=launch_setup),
    ])
