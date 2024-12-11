#!/usr/bin/env python3

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

from argparse import ArgumentParser
import datetime
import os
import shutil
import sys
import tempfile
import time
from types import SimpleNamespace

import data_processor as dp
import isaac_ros_launch_utils as lu
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


H264_IMAGE_MSG_TYPE = 'sensor_msgs/msg/CompressedImage'
H264_IMAGE_SUFFIX = '/image_compressed'
UDP_PACKETS_MSG_TYPE = 'hesai_ros_driver/msg/UdpFrame'
UDP_PACKETS_SUFFIX = '/lidar_packets'


def process_topic(input_rosbag, output_rosbag, launch_file, launch_arguments, passthrough=True):
    launch_description = LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file),
            launch_arguments=launch_arguments.items(),
        )
    ])

    data_processor = dp.DataProcessor(input_handler=dp.RosbagReader(input_rosbag),
                                      output_handler=dp.RosbagWriter(output_rosbag))
    data_processor.run(launch_description=launch_description, passthrough=passthrough)


def convert_bag(input_rosbag, output_rosbag, options_list):
    """Convert a rosbag using the list of options assembled by get_processing_options_list."""
    number_conversions = len(options_list)

    def print_status(start_time, elapsed_time, idx):
        if elapsed_time:
            remaining_time_sec = (elapsed_time - start_time) / (idx) * (number_conversions - idx)
            remaining_time_str = str(datetime.timedelta(seconds=round(remaining_time_sec)))
            remaining_time_msg = f'Remaining time [hh:mm:ss]: {remaining_time_str}'
        else:
            remaining_time_msg = 'Estimating remaining time...'
        print(f' - Converting {idx+1} / {number_conversions}... ({remaining_time_msg})')

    input_rosbag_temp = input_rosbag
    with tempfile.TemporaryDirectory() as temp_directory_name:
        delete_rosbag = None
        start_time = time.time()
        elapsed_time = None
        # Process all but last namespace in list, and output temporary rosbags in temporary dirs
        for idx, opts in enumerate(options_list[:-1]):
            print_status(start_time, elapsed_time, idx)
            output_rosbag_tmp = os.path.join(temp_directory_name, str(idx))
            process_topic(input_rosbag_temp, output_rosbag_tmp, opts.launchfile, opts.args)
            if delete_rosbag and delete_rosbag != input_rosbag:
                shutil.rmtree(delete_rosbag)
            delete_rosbag = input_rosbag_temp
            input_rosbag_temp = output_rosbag_tmp
            elapsed_time = time.time()

        if delete_rosbag and delete_rosbag != input_rosbag:
            shutil.rmtree(delete_rosbag)

        # Process the last namespace in the list and output directly to output_rosbag
        print_status(start_time, elapsed_time, number_conversions-1)
        process_topic(input_rosbag_temp, output_rosbag, options_list[-1].launchfile,
                      options_list[-1].args)


def get_processing_options_list(namespaces_decode_h264, namespaces_udp_to_pc2):
    """Return list of launchfile-launch arguments pairs to be provided to convert_bag function."""
    options_list = []
    for namespace in namespaces_decode_h264:
        launch_path = lu.get_path('isaac_ros_data_replayer', 'launch/include/decoder.launch.py')
        launch_args = {'namespace': namespace}
        options_list.append(SimpleNamespace(launchfile=launch_path.as_posix(), args=launch_args))

    for namespace in namespaces_udp_to_pc2:
        launch_path = lu.get_path('isaac_ros_hesai', 'launch/hesai.launch.py')
        launch_args = {'namespace': namespace, 'replay': 'True'}
        options_list.append(SimpleNamespace(launchfile=launch_path.as_posix(), args=launch_args))
    return options_list


def topics_to_namespaces(topics_list, topic_suffix):
    """
    Parse the namespace part of a topic string, for each topic in the input list.

    Return the list of namespace for a given list of topic, following the naming format
    /<namespace><topic_suffix> (e.g., 'right_fisheye_camera/left' for the topic
    /right_fisheye_camera/left/image_compressed).
    """
    return [topic.removesuffix(topic_suffix).removeprefix('/') for topic in topics_list]


def get_topics_by_msg_type(input_rosbag, msg_type):
    """Return all topics in a robag with a given message type."""
    with dp.RosbagReader(input_rosbag) as rosbag_reader:
        topics_types = rosbag_reader.topics()
        return [topic_type.name for topic_type in topics_types if topic_type.type == msg_type]


def get_namespaces_by_msg_type(input_rosbag, msg_type, topic_suffix):
    """
    Figure out list of all possible namespaces to process with this script by parsing bag topics.

    Return the list of namespaces in the input_rosbag that are publishing messages in 'msg_type'
    and following the naming format /<namespace><topic_suffix> (e.g., 'right_fisheye_camera/left'
    for the topic /right_fisheye_camera/left/image_compressed).
    """
    return topics_to_namespaces(get_topics_by_msg_type(input_rosbag, msg_type), topic_suffix)


def check_topics_in_bag(input_rosbag, msg_type, topics_list):
    """Verify that the provided topics are in the input rosbag and use the given message type."""
    topics_in_bag = get_topics_by_msg_type(input_rosbag, msg_type)
    if not set(topics_list).issubset(set(topics_in_bag)):
        raise ValueError(f'One or more provided topics ({topics_list}) cannot be found in the '
                         f'input rosbag ({input_rosbag}) with message type ({msg_type}). '
                         f'Available topics: {topics_in_bag}')


def parse_args():
    parser = ArgumentParser(prog='rosbag_converter.py')
    parser.add_argument('--input_rosbag',
                        '-i',
                        help='input rosbag path',
                        type=str,
                        required=True)
    parser.add_argument('--output_rosbag',
                        '-o',
                        help='output rosbag path',
                        type=str,
                        required=True)
    parser.add_argument('--all',
                        '-a',
                        help='Whether to convert all possible topics in the input rosbag. If '
                        'provided, it takes priority over other arguments listing namespaces',
                        action='store_true')
    parser.add_argument('--topics_decode_h264',
                        help='List of image topics with CompressedImage h264 to be decoded',
                        nargs='+',
                        default=[],
                        required=False)
    parser.add_argument('--topics_udp_to_pc2',
                        help='List of lidar topics with udp packets to be converted to pc2',
                        nargs='+',
                        default=[],
                        required=False)
    return parser.parse_args()


def main():
    args = parse_args()

    if not os.path.exists(args.input_rosbag):
        print('Input path does not exist')
        sys.exit(1)

    if os.path.exists(args.output_rosbag):
        print('Output path already exist')
        sys.exit(1)

    if args.all:
        namespaces_decode_h264 = get_namespaces_by_msg_type(
            args.input_rosbag, H264_IMAGE_MSG_TYPE, H264_IMAGE_SUFFIX)
        namespaces_udp_to_pc2 = get_namespaces_by_msg_type(
            args.input_rosbag, UDP_PACKETS_MSG_TYPE, UDP_PACKETS_SUFFIX)

        if args.topics_decode_h264 or args.topics_udp_to_pc2:
            print('Warning: --all flag passed. Overwriting the provided namespaces list(s) with:')
    else:
        check_topics_in_bag(args.input_rosbag, H264_IMAGE_MSG_TYPE, args.topics_decode_h264)
        check_topics_in_bag(args.input_rosbag, UDP_PACKETS_MSG_TYPE, args.topics_udp_to_pc2)
        namespaces_decode_h264 = topics_to_namespaces(args.topics_decode_h264, H264_IMAGE_SUFFIX)
        namespaces_udp_to_pc2 = topics_to_namespaces(args.topics_udp_to_pc2, UDP_PACKETS_SUFFIX)
    print(f'- namespaces_decode_h264: {namespaces_decode_h264}')
    print(f'- namespaces_udp_to_pc2: {namespaces_udp_to_pc2}')

    options_list = get_processing_options_list(namespaces_decode_h264, namespaces_udp_to_pc2)

    if len(options_list) == 0:
        print('Nothing to do. Input namespaces lists are empty.')
        sys.exit(0)

    convert_bag(args.input_rosbag, args.output_rosbag, options_list)
    print(' - rosbag conversion done!')


if __name__ == '__main__':
    main()
