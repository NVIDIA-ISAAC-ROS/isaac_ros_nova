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

from foxglove_msgs.msg import CompressedVideo
from rclpy.serialization import deserialize_message, serialize_message
import rosbag2_py
from sensor_msgs.msg import CompressedImage


def compressed_image_to_compressed_video(compressed_image_msg):
    compressed_video_msg = CompressedVideo()
    compressed_video_msg.timestamp = compressed_image_msg.header.stamp
    compressed_video_msg.frame_id = compressed_image_msg.header.frame_id
    compressed_video_msg.data = compressed_image_msg.data
    compressed_video_msg.format = compressed_image_msg.format
    return compressed_video_msg


def rosbag_writer(uri, storage_id='mcap', serialization_format='cdr'):
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(
            uri=uri,
            storage_id=storage_id),
        rosbag2_py.ConverterOptions(
            input_serialization_format=serialization_format,
            output_serialization_format=serialization_format,
        ),
    )
    return writer


def rosbag_reader(uri, storage_id='mcap', serialization_format='cdr'):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(
            uri=uri,
            storage_id=storage_id,
        ),
        rosbag2_py.ConverterOptions(
            input_serialization_format=serialization_format,
            output_serialization_format=serialization_format,
        ),
    )
    return reader


def parse_args():
    parser = ArgumentParser(prog='data_extraction.py')
    parser.add_argument('input', help='path to input rosbag')
    parser.add_argument('output', help='path to output rosbag')
    return parser.parse_args()


def main():
    args = parse_args()

    reader = rosbag_reader(args.input)
    writer = rosbag_writer(args.output)

    topics = reader.get_all_topics_and_types()
    for topic in topics:
        topic_name = topic.name.replace(
            'image_compressed', 'video_compressed'
        )
        topic_type = topic.type.replace(
            'sensor_msgs/msg/CompressedImage', 'foxglove_msgs/msg/CompressedVideo'
        )
        writer.create_topic(
            rosbag2_py.TopicMetadata(name=topic_name, type=topic_type, serialization_format='cdr')
        )

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if 'image_compressed' in topic:
            compressed_image_msg = deserialize_message(data, CompressedImage)
            compressed_video_msg = compressed_image_to_compressed_video(compressed_image_msg)
            topic = topic.replace('image_compressed', 'video_compressed')
            data = serialize_message(compressed_video_msg)
        writer.write(topic, data, timestamp)


if __name__ == '__main__':
    main()
