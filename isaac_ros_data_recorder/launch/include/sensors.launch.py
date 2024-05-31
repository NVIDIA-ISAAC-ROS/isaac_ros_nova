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
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, Shutdown
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import isaac_ros_launch_utils as lu


def load_config(context: LaunchContext,
                target_container: LaunchConfiguration,
                config: LaunchConfiguration):
    isaac_ros_data_recorder_launch_include_dir = os.path.join(
        get_package_share_directory('isaac_ros_data_recorder'), 'launch', 'include')

    nova_carter_description_launch_dir = os.path.join(
        get_package_share_directory('nova_carter_description'), 'launch')

    nova_developer_kit_description_launch_dir = os.path.join(
        get_package_share_directory('nova_developer_kit_description'), 'launch')

    isaac_ros_correlated_timestamp_driver_launch_dir = os.path.join(
        get_package_share_directory('isaac_ros_correlated_timestamp_driver'), 'launch')

    isaac_ros_hesai_launch_dir = os.path.join(
        get_package_share_directory('isaac_ros_hesai'), 'launch')

    isaac_ros_imu_bmi088_launch_dir = os.path.join(
        get_package_share_directory('isaac_ros_imu_bmi088'), 'launch')

    actions = []
    actions.append(
        Node(
            name=target_container,
            package='rclcpp_components',
            executable='component_container_mt',
            output='screen',
            on_exit=Shutdown(),
            condition=LaunchConfigurationEquals('target_container', 'sensors'),
        ))

    # Launch robot description depending on nova robot type
    nova_robot = lu.get_nova_robot()
    if nova_robot is lu.NovaRobot.NOVA_CARTER:
        actions.append(
            lu.include('nova_carter_description', 'launch/nova_carter_description.launch.py'))
    elif nova_robot is lu.NovaRobot.NOVA_DEVELOPER_KIT:
        actions.append(
            lu.include('nova_developer_kit_description',
                       'launch/nova_developer_kit_description.launch.py'))
    else:
        actions.append(
            lu.log_info([
                "NovaRobot type ", nova_robot.name,
                " not implemented for data recorder. Launching without robot description.'"
            ]))

    actions.append(
        lu.include(
            'isaac_ros_correlated_timestamp_driver',
            'launch/correlated_timestamp_driver.launch.py',
            launch_arguments={'target_container': target_container},
        ))


    with (open(context.perform_substitution(config), 'r') as file,
          open('/etc/nova/systeminfo.yaml', 'r') as systeminfo):
        app_config = yaml.safe_load(file)
        system_config = yaml.safe_load(systeminfo)

        for sensor in app_config['sensors']:
            if sensor not in system_config['sensors']:
                raise KeyError("Sensor ' " + sensor + "' in " + file.name +
                               " not found in " + systeminfo.name)

            sensor_config = system_config['sensors'][sensor]

            if sensor_config['type'] == 'hawk':
                if 'module_id' in sensor_config:
                    hawk_launch = IncludeLaunchDescription(
                        PythonLaunchDescriptionSource([
                            isaac_ros_data_recorder_launch_include_dir,
                            '/hawk.launch.py'
                        ]),
                        launch_arguments={
                            'target_container': target_container,
                            'namespace': sensor,
                            'module_id': str(sensor_config['module_id']),
                        }.items(),
                    )
                    actions.append(hawk_launch)
                else:
                    # remove after nova-init updates type 'front_stereo_imu'
                    bmi088_launch = IncludeLaunchDescription(
                        PythonLaunchDescriptionSource([
                            isaac_ros_imu_bmi088_launch_dir,
                            '/bmi088.launch.py'
                        ]),
                        launch_arguments={
                            'target_container': target_container,
                            'namespace': sensor,
                        }.items(),
                    )
                    actions.append(bmi088_launch)
            elif sensor_config['type'] == 'owl':
                owl_launch = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        isaac_ros_data_recorder_launch_include_dir,
                        '/owl.launch.py'
                    ]),
                    launch_arguments={
                        'target_container': target_container,
                        'namespace': sensor,
                        'camera_id': str(sensor_config['camera_id']),
                    }.items(),
                )
                actions.append(owl_launch)
            elif sensor_config['type'] == 'rplidar':
                rplidar_launch = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        isaac_ros_data_recorder_launch_include_dir,
                        '/rplidar.launch.py'
                    ]),
                    launch_arguments={
                        'name': sensor,
                        'udp_ip': sensor_config['ip'],
                    }.items(),
                )
                actions.append(rplidar_launch)
            elif sensor_config['type'] == 'hesai':
                hesai_launch = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        isaac_ros_hesai_launch_dir,
                        '/hesai.launch.py'
                    ]),
                    launch_arguments={
                        'namespace': sensor,
                    }.items(),
                )
                actions.append(hesai_launch)
            elif sensor_config['type'] == 'bmi088':
                bmi088_launch = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        isaac_ros_imu_bmi088_launch_dir,
                        '/bmi088.launch.py'
                    ]),
                    launch_arguments={
                        'target_container': target_container,
                        'namespace': sensor,
                    }.items(),
                )
                actions.append(bmi088_launch)

    return [LaunchDescription(actions)]


def generate_launch_description():
    target_container = LaunchConfiguration('target_container')
    target_container_launch_arg = DeclareLaunchArgument(
        'target_container',
        description='Target Container',
        default_value='sensors',
    )

    config = LaunchConfiguration('config')
    config_launch_arg = DeclareLaunchArgument(
        'config',
        description='Sensor Configuration File',
        default_value='/etc/nova/systeminfo.yaml',
    )

    launch_args = [
        target_container_launch_arg,
        config_launch_arg,
    ]

    return LaunchDescription(launch_args + [
        OpaqueFunction(function=load_config, args=[target_container, config]),
    ])
