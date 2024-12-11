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

from typing import List

import isaac_ros_launch_utils as lu


DEFAULT_CONTAINER = 'data_recorder'


def rename_topic(topic: str, base: str) -> str:
    return topic.replace(topic.split('/')[-1], base)


def resize_node_description(name: str,
                            namespace: str,
                            image_topic: str,
                            camera_info_topic: str,
                            output_width: int,
                            output_height: int) -> lu.ComposableNode:
    image_resized_topic = rename_topic(image_topic, 'image_resized')
    camera_info_resized_topic = camera_info_topic + '_resized'
    resize_node = lu.ComposableNode(
        name=name,
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ResizeNode',
        namespace=namespace,
        remappings=[
            ('image', image_topic),
            ('camera_info', camera_info_topic),
            ('resize/image', image_resized_topic),
            ('resize/camera_info', camera_info_resized_topic),
        ],
        parameters=[{
            'output_width': output_width,
            'output_height': output_height,
            'type_negotiation_duration_s': 5,
            'image_nitros_format': 'nitros_image_nv12',
            'resize/image_nitros_format': 'nitros_image_nv12',
        }],
    )
    return resize_node


def encoder_node_description(name: str,
                             namespace: str,
                             image_topic: str,
                             input_width: int,
                             input_height: int,
                             encoder_qp: int
                             ) -> lu.ComposableNode:
    image_compressed_topic = rename_topic(image_topic, 'image_compressed')

    encoder_node = lu.ComposableNode(
        name=name,
        package='isaac_ros_h264_encoder',
        plugin='nvidia::isaac_ros::h264_encoder::EncoderNode',
        namespace=namespace,
        remappings=[
            ('image_raw', image_topic),
            ('image_compressed', image_compressed_topic),
        ],
        parameters=[{
            'input_width': input_width,
            'input_height': input_height,
            'config': 'custom',
            'qp': encoder_qp,
            'type_negotiation_duration_s': 5,
            'image_raw_nitros_format': 'nitros_image_nv12',
        }]
    )
    return encoder_node


def data_recorder_launch_description(args: lu.ArgumentContainer) -> List[lu.Action]:
    nodes = []
    topics = args.topics
    for sensor, settings in args.sensors.items():
        if sensor.endswith('stereo_camera'):
            if 'width' in settings and 'height' in settings:
                width = settings['width']
                height = settings['height']
            else:
                width = 1920
                height = 1200

            resized = width != 1920 or height != 1200
            if resized:
                nodes.append(
                    resize_node_description(
                        name='left_resize',
                        namespace=sensor,
                        image_topic='left/image_raw',
                        camera_info_topic='left/camera_info',
                        output_width=width,
                        output_height=height,
                    )
                )

                if not args.mono:
                    nodes.append(
                        resize_node_description(
                            name='right_resize',
                            namespace=sensor,
                            image_topic='right/image_raw',
                            camera_info_topic='right/camera_info',
                            output_width=width,
                            output_height=height,
                        )
                    )

            nodes.append(
                encoder_node_description(
                    name='left_encoder',
                    namespace=sensor,
                    image_topic='left/image_' + ('resized' if resized else 'raw'),
                    input_width=width,
                    input_height=height,
                    encoder_qp=args.encoder_qp,
                )
            )

            if not args.mono:
                nodes.append(
                    encoder_node_description(
                        name='right_encoder',
                        namespace=sensor,
                        image_topic='right/image_' + ('resized' if resized else 'raw'),
                        input_width=width,
                        input_height=height,
                        encoder_qp=args.encoder_qp,
                    )
                )

            topics.append('/' + sensor + '/left/image_compressed')
            topics.append('/' + sensor + '/left/camera_info' + ('_resized' if resized else ''))
            if not args.mono:
                topics.append('/' + sensor + '/right/image_compressed')
                topics.append('/' + sensor + '/right/camera_info' +
                              ('_resized' if resized else ''))
        elif sensor.endswith('fisheye_camera'):
            if 'width' in settings and 'height' in settings:
                width = settings['width']
                height = settings['height']
            else:
                width = 1920
                height = 1200

            resized = width != 1920 or height != 1200
            if resized:
                nodes.append(
                    resize_node_description(
                        name='resize',
                        namespace=sensor,
                        image_topic='left/image_raw',
                        camera_info_topic='left/camera_info',
                        output_width=width,
                        output_height=height,
                    )
                )

            nodes.append(
                encoder_node_description(
                    name='encoder',
                    namespace=sensor,
                    image_topic='left/image_' + ('resized' if resized else 'raw'),
                    input_width=width,
                    input_height=height,
                    encoder_qp=args.encoder_qp,
                )
            )

            topics.append('/' + sensor + '/left/image_compressed')
            topics.append('/' + sensor + '/left/camera_info' + ('_resized' if resized else ''))
        elif sensor.endswith('2d_lidar'):
            topics.append('/' + sensor + '/scan')
        elif sensor.endswith('3d_lidar'):
            topics.append('/' + sensor + '/lidar_packets')
        elif sensor.endswith('imu'):
            topics.append('/' + sensor + '/imu')
        elif sensor in ('chassis', 'drivetrain'):
            for channel, enabled in settings.items():
                if channel in ('imu', 'ticks', 'odom', 'battery_state') and enabled:
                    topics.append('/chassis/' + channel)

    actions = []

    if args.target_container == DEFAULT_CONTAINER:
        actions.append(
            lu.component_container(
                container_name=args.target_container,
            )
        )

    if args.event_recorder:
        actions.append(
            lu.Node(
                name='event_recorder',
                package='isaac_ros_data_recorder',
                executable='event_recorder_node.py',
                parameters=[{
                    'topics': topics,
                    'recording_directory': args.recording_directory,
                    'recording_name': args.recording_name,
                    'max_bag_duration': args.max_bag_duration,
                    'look_back_window': args.look_back_window,
                    'look_ahead_window': args.look_ahead_window,
                }],
            )
        )
    else:
        actions.append(
            lu.Node(
                name='rosbag2',
                package='isaac_ros_data_recorder',
                executable='rosbag2_node.py',
                parameters=[{
                    'topics': topics,
                    'files': args.files,
                    'recording_directory': args.recording_directory,
                    'recording_name': args.recording_name,
                    'prepend_datetime': args.prepend_datetime,
                    'enable_services': args.enable_services,
                }],
                output='both',
                on_exit=lu.Shutdown(),
            )
        )

    if nodes:
        actions.append(lu.load_composable_nodes(args.target_container, nodes))

    return actions


def generate_launch_description():
    args = lu.ArgumentContainer()
    args.add_arg('target_container',
                 description='target container',
                 default=DEFAULT_CONTAINER,
                 cli=True)
    args.add_arg('sensors',
                 description='sensor recording configuration',
                 default={},
                 cli=True)
    args.add_arg('topics',
                 description='additional topics to record',
                 default=['--all'],
                 cli=True)
    args.add_arg('files',
                 description='files to record',
                 default=[''],
                 cli=True)
    args.add_arg('recording_directory',
                 description='recording directory',
                 default='.',
                 cli=True)
    args.add_arg('recording_name',
                 description='recording name',
                 default='rosbag2',
                 cli=True)
    args.add_arg('prepend_datetime',
                 description='prepend datetime to recording name',
                 default=True,
                 cli=True)
    args.add_arg('enable_services',
                 description='enable services to start and stop recordings',
                 default=True,
                 cli=True)
    args.add_arg('mono',
                 description='record stereo cameras in mono for calibration',
                 default=False,
                 cli=True)
    args.add_arg('event_recorder',
                 description='enable event recording',
                 default=False,
                 cli=True)
    args.add_arg('max_bag_duration',
                 description='event recorder max bag duration in seconds',
                 default=60,
                 cli=True)
    args.add_arg('look_back_window',
                 description='event recorder look-back window duration in seconds',
                 default=60,
                 cli=True)
    args.add_arg('look_ahead_window',
                 description='event recorder look-ahead window duration in seconds',
                 default=60,
                 cli=True)
    args.add_arg('encoder_qp',
                 description='H.264 encoder quantization parameter',
                 default=20,
                 cli=True)
    args.add_opaque_function(data_recorder_launch_description)

    return lu.LaunchDescription(args.get_launch_actions())
