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

from pathlib import Path
import shutil
from typing import List

import isaac_ros_launch_utils as lu
import yaml

DEFAULT_CONTAINER = 'nova_recorder'
RECORDING_CONFIG = '/tmp/recording_config.yaml'


def nova_recorder_launch_description(args: lu.ArgumentContainer) -> List[lu.Action]:
    actions = []

    config_path = Path(args.config)
    if not config_path.is_file():
        default_dir = lu.get_path('isaac_ros_nova', 'config')
        default_path = default_dir.joinpath(config_path)
        # append .yaml if there is no suffix
        if not default_path.suffix:
            default_path = default_path.with_suffix('.yaml')
        if not default_path.is_file():
            ls_configs = [entry.stem for entry in default_dir.iterdir() if entry.is_file()]
            ls_configs_str = ' '.join(ls_configs)
            raise ValueError(
                f'Tried to find YAML {args.config} but failed. Use a valid file '
                f'path or one of the available YAMLs: {ls_configs_str}'
            )
        config_path = default_path

    shutil.copyfile(config_path, RECORDING_CONFIG)

    sensors = {}
    topics = args.topics + [
        '/rosout',
        '/diagnostics',
        '/diagnostics_agg',
        '/tf',
        '/tf_static',
        '/robot_description',
    ]
    with open(RECORDING_CONFIG, 'r') as file:
        config = yaml.safe_load(file)
        if config and 'sensors' in config and config['sensors']:
            for sensor in config['sensors']:
                if sensor.endswith('camera'):
                    sensors[sensor] = {
                        'width': 1920,
                        'height': 1200,
                    }
                    if '/correlated_timestamp' not in topics:
                        topics.append('/correlated_timestamp')
                elif sensor in ('chassis', 'drivetrain'):
                    sensors[sensor] = {
                        'imu': True,
                        'ticks': True,
                        'odom': True,
                        'battery_state': True,
                    }

                    actions.append(
                        lu.include(
                            package='nova_carter_bringup',
                            path='launch/include/teleop_include.launch.py',
                            launch_arguments={
                                'enable_wheel_odometry': args.enable_wheel_odometry,
                            },
                        ))
                else:
                    sensors[sensor] = {}

    if args.target_container == DEFAULT_CONTAINER:
        actions.append(
            lu.component_container(
                container_name=args.target_container,
            )
        )

    actions.append(
        lu.include(
            package='isaac_ros_nova',
            path='launch/nova.launch.py',
            launch_arguments={
                'target_container': args.target_container,
                'config': args.config,
            },
        )
    )

    files = [
        RECORDING_CONFIG,
        '/etc/nova/systeminfo.yaml',
        '/etc/nova/metadata.json',
        '/etc/nova/calibration/isaac_nominals.urdf',
        '/etc/nova/calibration/isaac_calibration.urdf',
    ]

    if 'front_3d_lidar' in sensors:
        files.append('/tmp/hesai/correction.csv')

    actions.append(
        lu.include(
            package='isaac_ros_data_recorder',
            path='launch/data_recorder.launch.py',
            launch_arguments={
                'target_container': args.target_container,
                'sensors': sensors,
                'topics': topics,
                'files': files,
                'recording_directory': args.recording_directory,
                'enable_services': not args.headless,
                'event_recorder': args.event_recorder,
                'encoder_qp': args.encoder_qp,
            }
        )
    )

    if not args.headless:
        actions.append(
            lu.Node(
                executable='foxglove_bridge',
                parameters=[{
                    'send_buffer_limit': 100000,
                    'max_qos_depth': 1,
                    'use_compression': True,
                    'include_hidden': True,
                }],
                arguments=['--ros-args', '--log-level', 'ERROR'],
                package='foxglove_bridge',
            )
        )

        actions.append(
            lu.Node(
                name='rosbag_manager',
                package='isaac_ros_nova_recorder',
                executable='rosbag_manager_node.py',
                parameters=[{
                    'directory': args.recording_directory,
                }],
            )
        )

        actions.append(
            lu.Node(
                name='rosbag_uploader',
                package='isaac_ros_nova_recorder',
                executable='rosbag_uploader_node.py',
                parameters=[{
                    'directory': args.recording_directory,
                    'bucket': args.s3_bucket,
                }],
            )
        )

        actions.append(
            lu.Node(
                name='web_server',
                package='isaac_ros_nova_recorder',
                executable='web_server.py',
                output='screen',
                parameters=[{
                    'webserver_port': args.webserver_port,
                    'retry_count': args.retry_count,
                    'retry_delay': args.retry_delay
                }]
            )
        )

        cameras = []
        for sensor, settings in sensors.items():
            if sensor.endswith('stereo_camera'):
                cameras.append('/' + sensor + '/left')
                cameras.append('/' + sensor + '/right')
            elif sensor.endswith('fisheye_camera'):
                cameras.append('/' + sensor + '/left')

        if len(cameras) > 0:
            nodes = []

            nodes.append(
                lu.ComposableNode(
                    name='camera_multiplexer',
                    package='isaac_ros_nova_recorder',
                    plugin='nvidia::isaac_ros::isaac_ros_nova_recorder::CameraMultiplexerNode',
                    namespace='preview',
                    parameters=[{
                        'cameras': cameras,
                        'select': cameras[0],
                        'type_negotiation_duration_s': 5,
                    }],
                )
            )

            nodes.append(
                lu.ComposableNode(
                    name='resize',
                    package='isaac_ros_image_proc',
                    plugin='nvidia::isaac_ros::image_proc::ResizeNode',
                    namespace='preview',
                    remappings=[
                        ('image', 'image_raw'),
                        ('camera_info', 'camera_info'),
                        ('resize/image', 'image_resized'),
                        ('resize/camera_info', 'camera_info_resized'),
                    ],
                    parameters=[{
                        'output_width': 480,
                        'output_height': 300,
                        'type_negotiation_duration_s': 5,
                        'image_nitros_format': 'nitros_image_nv12',
                        'resize/image_nitros_format': 'nitros_image_nv12',
                    }],
                )
            )

            nodes.append(
                lu.ComposableNode(
                    name='encoder',
                    package='isaac_ros_h264_encoder',
                    plugin='nvidia::isaac_ros::h264_encoder::EncoderNode',
                    namespace='preview',
                    remappings=[
                        ('image_raw', 'image_resized'),
                    ],
                    parameters=[{
                        'input_width': 480,
                        'input_height': 300,
                        'type_negotiation_duration_s': 5,
                    }],
                )
            )

            actions.append(lu.load_composable_nodes(args.target_container, nodes))

        if args.event_recorder and ('chassis' in sensors or 'drivetrain' in sensors):
            actions.append(
                lu.Node(
                    name='event_trigger',
                    package='isaac_ros_nova_recorder',
                    executable='event_trigger_node.py',
                    remappings=[
                        ('joy', 'joy/joy'),
                    ],
                )
            )

    return actions


def generate_launch_description():
    args = lu.ArgumentContainer()
    args.add_arg('target_container',
                 description='target container',
                 default=DEFAULT_CONTAINER,
                 cli=True)
    args.add_arg('config',
                 description='sensor configuration file',
                 default='/etc/nova/systeminfo.yaml',
                 cli=True)
    args.add_arg('topics',
                 description='additional topics to record',
                 default=[],
                 cli=True)
    args.add_arg('recording_directory',
                 description='recording directory',
                 default='/mnt/nova_ssd/recordings',
                 cli=True)
    args.add_arg('s3_bucket',
                 description='S3 bucket',
                 default='',
                 cli=True)
    args.add_arg('enable_wheel_odometry',
                 description='enable segway wheel odometry',
                 default=True,
                 cli=True)
    args.add_arg('headless',
                 description='record data without the UI',
                 default=False,
                 cli=True)
    args.add_arg('event_recorder',
                 description='enable event recording',
                 default=False,
                 cli=True)
    args.add_arg('webserver_port',
                 description='webserver port for the UI',
                 default=8080,
                 cli=True)
    args.add_arg('retry_count',
                 description='Number of retries if the server fails to start',
                 default=100,
                 cli=True)
    args.add_arg('retry_delay',
                 description='Delay in seconds between retries',
                 default=2.0,
                 cli=True)
    args.add_arg('encoder_qp',
                 description='H.264 encoder quality parameter',
                 default=20,
                 cli=True)
    args.add_opaque_function(nova_recorder_launch_description)

    return lu.LaunchDescription(args.get_launch_actions())
