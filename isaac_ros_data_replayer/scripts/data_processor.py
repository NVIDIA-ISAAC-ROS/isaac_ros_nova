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
import json
import multiprocessing
import os
import pydoc
from signal import SIGINT
import time

from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message, serialize_message
from rosbag2_py import ConverterOptions, StorageOptions, TopicMetadata
from rosbag2_py import SequentialReader, SequentialWriter


class RosPipeline:
    """Runs a ROS 2 launch file in a seperate process."""

    def __init__(self, launch_description: LaunchDescription):
        self.process = multiprocessing.Process(
            target=RosPipeline.launch,
            args=(launch_description,),
        )

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exception_type, exception_value, exception_traceback):
        self.stop()

    def start(self):
        self.process.start()

    def stop(self):
        os.kill(self.process.pid, SIGINT)
        self.process.join()

    def launch(launch_description: LaunchDescription):
        launch_service = LaunchService()
        launch_service.include_launch_description(launch_description)
        launch_service.run()
        launch_service.shutdown()


class DataProcessor:

    class InputHandler():
        """Default input handler for DataProcessor that does nothing."""

        def __enter__(self):
            return self

        def __exit__(self, exception_type, exception_value, exception_traceback):
            pass

        def topics(self):
            return None

        def available(self):
            return True

        def read(self):
            return None, None, None

    class OutputHandler():
        """Default output handler for DataProcessor that does nothing."""

        def __enter__(self):
            return self

        def __exit__(self, exception_type, exception_value, exception_traceback):
            pass

        def create_topic(self, topic_name, topic_type):
            pass

        def write(self, topic, data, timestamp):
            pass

    def __init__(self,
                 input_handler: InputHandler = InputHandler(),
                 output_handler: OutputHandler = OutputHandler()) -> None:
        self.input_handler = input_handler
        self.output_handler = output_handler
        self.lock = multiprocessing.Lock()
        self.msg = None
        self.topic = None

    def callback(self, msg, topic: str) -> None:
        with self.lock:
            self.msg = msg
            self.topic = topic

    def create_publishers(self, node: Node) -> dict:
        publishers = {}

        for node_name, node_namespace in node.get_node_names_and_namespaces():
            subscribers = node.get_subscriber_names_and_types_by_node(node_name, node_namespace)
            for topic_name, topic_types in subscribers:
                # create publishers for topics without subscribers
                # ignore '/parameter_events' and NITROS topics
                if topic_name in ('/parameter_events') or 'nitros' in topic_name:
                    continue
                elif node.count_publishers(topic_name) == 0:
                    topic_type = pydoc.locate(topic_types[0].replace('/', '.'))
                    publisher = node.create_publisher(topic_type, topic_name, 10)
                    publishers[topic_name] = topic_types[0], topic_type, publisher
        return publishers

    def create_subscribers(self, node: Node) -> dict:
        subscribers = {}

        for node_name, node_namespace in node.get_node_names_and_namespaces():
            publishers = node.get_publisher_names_and_types_by_node(node_name, node_namespace)
            for topic_name, topic_types in publishers:
                # create subscribers for topics without publishers
                # ignore '/parameter_events', '/rosout', and NITROS topics
                if topic_name in ('/parameter_events', '/rosout') or 'nitros' in topic_name:
                    continue
                elif node.count_subscribers(topic_name) == 0:
                    topic_type = pydoc.locate(topic_types[0].replace('/', '.'))
                    subscriber = node.create_subscription(
                        topic_type, topic_name,
                        lambda msg, topic=topic_name: self.callback(msg, topic), 10)
                    subscribers[topic_name] = topic_types[0], topic_type, subscriber

        return subscribers

    def run(self,
            launch_description: LaunchDescription,
            passthrough: bool = False,
            timeout: float = 1.0) -> None:
        rclpy.init()
        node = rclpy.create_node('data_processor')

        with self.input_handler, self.output_handler, RosPipeline(launch_description):
            # wait for pipeline to settle
            time.sleep(5)

            publishers = self.create_publishers(node)
            subscribers = self.create_subscribers(node)

            for topic_name in subscribers:
                topic_string, topic_type, subscriber = subscribers[topic_name]
                self.output_handler.create_topic(topic_name, topic_string)

            if passthrough:
                for topic in self.input_handler.topics():
                    if topic.name not in publishers:
                        self.output_handler.create_topic(topic.name, topic.type,
                                                         topic.offered_qos_profiles)

            with self.lock:
                self.msg = None
                self.topic = None

            while self.input_handler.available():
                topic, data, timestamp = self.input_handler.read()

                if topic in publishers:
                    topic_string, topic_type, publisher = publishers[topic]
                    msg = deserialize_message(data, topic_type)
                    publisher.publish(msg)
                    for i in range(len(subscribers)):
                        rclpy.spin_once(node, timeout_sec=timeout)

                        with self.lock:
                            if self.msg and self.topic:
                                data = serialize_message(self.msg)
                                self.output_handler.write(self.topic, data, timestamp)
                                self.msg = None
                                self.topic = None
                elif passthrough:
                    self.output_handler.write(topic, data, timestamp)

        node.destroy_node()
        rclpy.shutdown()


class RosbagReader(DataProcessor.InputHandler):
    """Wrapper for rosbag2_py.SequentialReader with context management."""

    def __init__(self, uri: str, storage_id: str = 'mcap', serialization_format: str = 'cdr'):
        super().__init__()
        self.uri = uri
        self.storage_id = storage_id
        self.serialization_format = serialization_format

    def __enter__(self):
        self.reader = SequentialReader()
        self.reader.open(
            StorageOptions(
                uri=self.uri,
                storage_id=self.storage_id,
            ),
            ConverterOptions(
                input_serialization_format=self.serialization_format,
                output_serialization_format=self.serialization_format,
            ),
        )
        return self

    def __exit__(self, exception_type, exception_value, exception_traceback):
        del self.reader

    def topics(self):
        return self.reader.get_all_topics_and_types()

    def available(self):
        return self.reader.has_next()

    def read(self):
        return self.reader.read_next()


class RosbagWriter(DataProcessor.OutputHandler):
    """Wrapper for rosbag2_py.SequentialWriter with context management."""

    def __init__(self, uri: str, storage_id: str = 'mcap', serialization_format: str = 'cdr'):
        super().__init__()
        self.uri = uri
        self.storage_id = storage_id
        self.serialization_format = serialization_format

    def __enter__(self):
        self.writer = SequentialWriter()
        self.writer.open(
            StorageOptions(
                uri=self.uri,
                storage_id=self.storage_id,
            ),
            ConverterOptions(
              input_serialization_format=self.serialization_format,
              output_serialization_format=self.serialization_format,
            ),
        )
        return self

    def __exit__(self, exception_type, exception_value, exception_traceback):
        del self.writer

    def create_topic(self, topic_name, topic_type, offered_qos_profiles=''):
        topic_metadata = TopicMetadata(
            name=topic_name,
            type=topic_type,
            serialization_format=self.serialization_format,
            offered_qos_profiles=offered_qos_profiles
        )
        self.writer.create_topic(topic_metadata)

    def write(self, topic, data, timestamp):
        self.writer.write(topic, data, timestamp)


def parse_args():
    parser = ArgumentParser(prog='data_processor.py')
    parser.add_argument('--input_rosbag',
                        help='input rosbag path',
                        type=str,
                        required=True)
    parser.add_argument('--output_rosbag',
                        help='output rosbag path',
                        type=str,
                        required=True)
    parser.add_argument('--launch_file',
                        help='launch file path',
                        type=str,
                        required=True)
    parser.add_argument('--launch_arguments',
                        help='launch arguments for launch file',
                        type=json.loads,
                        default={})
    parser.add_argument('--passthrough',
                        help='Forward unsubscribed topics from input to output',
                        action='store_true')
    return parser.parse_args()


def main():
    args = parse_args()

    launch_description = LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(args.launch_file),
            launch_arguments=args.launch_arguments.items(),
        )
    ])

    data_processor = DataProcessor(input_handler=RosbagReader(args.input_rosbag),
                                   output_handler=RosbagWriter(args.output_rosbag))
    data_processor.run(launch_description=launch_description, passthrough=args.passthrough)


if __name__ == '__main__':
    main()
