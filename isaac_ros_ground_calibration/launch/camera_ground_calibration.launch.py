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

import isaac_ros_launch_utils as lu
import isaac_ros_launch_utils.all_types as lut
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, OrSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

LOCAL_NAMESPACE = 'camera_ground_calibration'
DEFAULT_H264_COMPRESSED_TOPIC_SUFFIX = '/image_compressed'


def camera_info_topic_from_image_topic(image_topic, camera_info_suffix='/camera_info'):
    """Infer camera_info topic from a given image_topic, substituting suffix by camera_info."""
    idx_last_slash = image_topic.rfind('/')
    if idx_last_slash <= 0:
        raise ValueError('Failed to automatically infer camera_info topic from image_topic. '
                         'Please provide camera_info_topic argument')
    camera_info_topic = image_topic[:idx_last_slash] + camera_info_suffix
    return camera_info_topic


def launch_setup(context, *args, **kwargs):
    rosbag = LaunchConfiguration('rosbag')
    output = LaunchConfiguration('output')
    image_topic = LaunchConfiguration('image_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    target_file = LaunchConfiguration('target_file')
    h264_compressed_topic_suffix = LaunchConfiguration('h264_compressed_topic_suffix')
    rectify_width = LaunchConfiguration('rectify_width')
    rectify_height = LaunchConfiguration('rectify_height')

    # Check validity of input parameters
    rectify_width_str = rectify_width.perform(context)
    rectify_height_str = rectify_height.perform(context)
    if lu.is_none_or_null(rectify_width_str) != lu.is_none_or_null(rectify_height_str):
        raise ValueError('Please set both `rectify_width` and `rectify_width`, to rectify image '
                         'stream, or none of them if image stream is already rectified.')
    rectify = not lu.is_none_or_null(rectify_width_str)
    if rectify:
        if not rectify_width_str.isdigit() or not rectify_height_str.isdigit():
            raise ValueError('Both `rectify_width` and `rectify_width` must be positive integers.')

    # Choose topics to replay
    image_topic_str = context.perform_substitution(image_topic)
    if lu.is_none_or_null(camera_info_topic):
        camera_info_topic = camera_info_topic_from_image_topic(image_topic_str)
    replay_topics = [image_topic_str, camera_info_topic]
    replay_topics = ' '.join(replay_topics)

    play_rosbag = lu.play_rosbag(bag_path=rosbag,
                                 additional_bag_play_args=' '.join(['--topics', replay_topics]),
                                 condition=IfCondition(lu.is_valid(rosbag)))

    h264_compressed_topic_suffix_str = context.perform_substitution(h264_compressed_topic_suffix)
    decode_h264 = image_topic_str.endswith(h264_compressed_topic_suffix_str)
    if decode_h264:
        compressed_image_topic = image_topic
        uncompressed_image_topic = 'image_raw'
    else:
        compressed_image_topic = 'None'
        uncompressed_image_topic = image_topic

    if rectify:
        rectified_image_topic = 'image_rect'
        camera_info_rect_topic = 'camera_info_rect'
    else:
        rectified_image_topic = uncompressed_image_topic
        camera_info_rect_topic = camera_info_topic

    h264_decoder = ComposableNode(name='h264_decoder',
                                  package='isaac_ros_h264_decoder',
                                  plugin='nvidia::isaac_ros::h264_decoder::DecoderNode',
                                  namespace=LOCAL_NAMESPACE,
                                  remappings=[
                                      ('image_compressed', compressed_image_topic),
                                      ('image_uncompressed', uncompressed_image_topic),
                                  ],
                                  condition=IfCondition(str(decode_h264)))

    rectifier = ComposableNode(name='rectifier',
                               package='isaac_ros_image_proc',
                               plugin='nvidia::isaac_ros::image_proc::RectifyNode',
                               namespace=LOCAL_NAMESPACE,
                               remappings=[
                                   ('image_raw', uncompressed_image_topic),
                                   ('camera_info', camera_info_topic),
                                   ('image_rect', rectified_image_topic),
                                   ('camera_info_rect', camera_info_rect_topic),
                               ],
                               parameters=[{
                                   'output_width': rectify_width,
                                   'output_height': rectify_height,
                               }],
                               condition=IfCondition(str(rectify)))

    decoder_rectifier_container = ComposableNodeContainer(
        name='decoder_rectifier_container',
        package='rclcpp_components',
        executable='component_container_mt',
        namespace=LOCAL_NAMESPACE,
        composable_node_descriptions=[
            h264_decoder,
            rectifier,
        ],
        condition=IfCondition(OrSubstitution(str(rectify), str(decode_h264))),
    )

    plane_target_calibration = Node(
        name='plane_target_calibration',
        package='isaac_ros_ground_calibration',
        executable='plane_target_calibration.py',
        namespace=LOCAL_NAMESPACE,
        parameters=[{
            'target_description_path': target_file,
            'output_path': output,
            'shutdown_on_detection': True,
        }],
        remappings=[
            ('image_rect', rectified_image_topic),
            ('camera_info', camera_info_rect_topic),
        ],
        on_exit=lut.Shutdown(),
    )

    return [
        play_rosbag,
        decoder_rectifier_container,
        plane_target_calibration,
    ]


def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument(
            'image_topic',
            description='Image topic used for calibration. If the topic name ends in '
            f'{DEFAULT_H264_COMPRESSED_TOPIC_SUFFIX} (it can be changed with the '
            '"h264_compressed_topic_suffix" argument), it is assumed that the topic is compressed '
            'with H.264 and a decoder will be added to the graph.'),
        DeclareLaunchArgument(
            'h264_compressed_topic_suffix',
            default_value=DEFAULT_H264_COMPRESSED_TOPIC_SUFFIX,
            description='Topic suffix identifying topic names assumed to be encoded with H.264'),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='None',
            description='Camera info topic corresponding to image_topic. Default inferred from '
            'image_topic'),
        DeclareLaunchArgument('target_file',
                              description='Path to file YAML defining the calibration target.'),
        DeclareLaunchArgument('rosbag', description='Path to rosbag'),
        DeclareLaunchArgument('output',
                              default_value='',
                              description='Optional path to the output file'),
        DeclareLaunchArgument('rectify_width',
                              default_value='None',
                              description='Image width. To be set only if image is not already '
                              'rectified/undistorted.'),
        DeclareLaunchArgument('rectify_height',
                              default_value='None',
                              description='Image height. To be set only if image is not already '
                              'rectified/undistorted.'),
    ]

    return LaunchDescription(launch_args + [OpaqueFunction(function=launch_setup)])
