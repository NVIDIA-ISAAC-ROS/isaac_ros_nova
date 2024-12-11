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

from typing import List, Tuple
from unittest import TestCase

from launch import LaunchDescription
from launch_ros.actions import Node
from rclpy.serialization import serialize_message
from scripts.data_processor import DataProcessor
from sensor_msgs.msg import Image


class TestDataProcessor(TestCase):

    class InputHandler(DataProcessor.InputHandler):

        def __init__(self, msgs: List[Tuple[str, str, int]]):
            super().__init__()
            self.msgs = msgs
            self.index = 0

        def __enter__(self):
            self.index = 0
            return self

        def __exit__(self, exception_type, exception_value, exception_traceback):
            pass

        def topics(self):
            return None

        def available(self):
            return self.index < len(self.msgs)

        def read(self):
            topic, data, timestamp = self.msgs[self.index]
            self.index += 1
            return topic, data, timestamp

    class OutputHandler(DataProcessor.InputHandler):

        def __init__(self):
            self.msgs = []

        def __enter__(self):
            return self

        def __exit__(self, exception_type, exception_value, exception_traceback):
            pass

        def create_topic(self, topic_name, topic_type):
            pass

        def write(self, topic, data, timestamp):
            self.msgs.append((topic, data, timestamp))

    def run_test(input_topics: List[str],
                 output_topics: List[str],
                 msg_order: List[str]) -> List[Tuple[str, str, int]]:
        test_node = Node(
            name='test_node',
            package='isaac_ros_data_replayer',
            executable='test_node.py',
            parameters=[{
                'input_topics': input_topics,
                'output_topics': output_topics,
            }],
        )
        launch_description = LaunchDescription([test_node])

        msgs = []
        for index, msg in enumerate(msg_order):
            msgs.append((msg, serialize_message(Image()), index))

        input_handler = TestDataProcessor.InputHandler(msgs)
        output_handler = TestDataProcessor.OutputHandler()

        data_processor = DataProcessor(input_handler=input_handler, output_handler=output_handler)
        data_processor.run(launch_description)

        return output_handler.msgs

    def test_siso(self) -> None:
        input_topics = ['input0']
        output_topics = ['output0']
        msg_order = [
            '/input0',
        ]

        msgs = TestDataProcessor.run_test(input_topics, output_topics, msg_order)

        self.assertEqual(len(msgs), 1)

        topic, data, timestamp = msgs[0]
        self.assertEqual(topic, '/output0')
        self.assertEqual(timestamp, 0)

    def test_simo(self) -> None:
        input_topics = ['input0']
        output_topics = ['output0', 'output1']
        msg_order = [
            '/input0',
        ]

        msgs = TestDataProcessor.run_test(input_topics, output_topics, msg_order)

        self.assertEqual(len(msgs), 2)

        topic, data, timestamp = msgs[0]
        self.assertEqual(topic, '/output0')
        self.assertEqual(timestamp, 0)

        topic, data, timestamp = msgs[1]
        self.assertEqual(topic, '/output1')
        self.assertEqual(timestamp, 0)

    def test_miso(self) -> None:
        input_topics = ['input0', 'input1']
        output_topics = ['output0']
        msg_order = [
            '/input0',
            '/input1',
        ]

        msgs = TestDataProcessor.run_test(input_topics, output_topics, msg_order)

        self.assertEqual(len(msgs), 1)

        topic, data, timestamp = msgs[0]
        self.assertEqual(topic, '/output0')
        self.assertEqual(timestamp, 1)

    def test_mimo(self) -> None:
        input_topics = ['input0', 'input1']
        output_topics = ['output0', 'output1']
        msg_order = [
            '/input0',
            '/input1',
        ]

        msgs = TestDataProcessor.run_test(input_topics, output_topics, msg_order)

        self.assertEqual(len(msgs), 2)

        topic, data, timestamp = msgs[0]
        self.assertEqual(topic, '/output0')
        self.assertEqual(timestamp, 1)

        topic, data, timestamp = msgs[1]
        self.assertEqual(topic, '/output1')
        self.assertEqual(timestamp, 1)
