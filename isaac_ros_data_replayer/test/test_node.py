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

import multiprocessing

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class TestNode(Node):

    def __init__(self):
        super().__init__('test_node')

        self.declare_parameter('input_topics', rclpy.Parameter.Type.STRING_ARRAY)
        self.declare_parameter('output_topics', rclpy.Parameter.Type.STRING_ARRAY)

        self.inputs = []
        input_topics = self.get_parameter('input_topics').get_parameter_value()
        for input_topic in input_topics.string_array_value:
            subscriber = self.create_subscription(
                Image, input_topic,
                lambda msg, topic=input_topic: self.callback(msg, topic), 10)
            self.inputs.append(subscriber)

        self.outputs = []
        output_topics = self.get_parameter('output_topics').get_parameter_value()
        for output_topic in output_topics.string_array_value:
            publisher = self.create_publisher(Image, output_topic, 10)
            self.outputs.append(publisher)

        self.lock = multiprocessing.Lock()
        self.msgs = {}

    def callback(self, msg, topic):
        with self.lock:
            if topic not in self.msgs:
                self.msgs[topic] = msg
            if len(self.msgs) == len(self.inputs):
                for publisher in self.outputs:
                    publisher.publish(Image())
                self.msgs.clear()


def main():
    rclpy.init()
    rclpy.spin(TestNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
