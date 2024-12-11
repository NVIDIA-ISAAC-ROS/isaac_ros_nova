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

from datetime import datetime
import os
import re
import subprocess
import time
from typing import Dict, List
import uuid

from isaac_ros_data_recorder.msg import RecordingInfo
from isaac_ros_data_recorder.srv import StartRecording, StopRecording
import rclpy
from rclpy.node import Node


def mcap_add_metadata(metadata: Dict[str, str], path: str):
    keys = []
    for key, value in metadata.items():
        if key and value and key != 'timestamp' and key != 'duration':
            keys.extend(['--key', key + '=' + value])
    return mcap_add(['metadata', '--name', 'isaac_ros_data_recorder'] + keys, path)


def mcap_add_attachment(file: str, path: str):
    return mcap_add(['attachment', '--file', file], path)


def mcap_add(args: List[str], path: str):
    return subprocess.run(['mcap', 'add'] + args + [path], check=True)


def data_validation(path: str, verbosity: str = 'compact'):
    args = ['python', '-m', 'isaac_ros_data_validation.summarize_bag', '-v', verbosity, path]
    return subprocess.run(args, capture_output=True, text=True)


class Rosbag2Node(Node):

    def __init__(self):
        super().__init__('rosbag2')

        self.declare_parameter('topics', ['--all'])
        topics = self.get_parameter('topics')
        self.topics = topics.get_parameter_value().string_array_value

        self.declare_parameter('files', [''])
        files = self.get_parameter('files')
        self.files = files.get_parameter_value().string_array_value

        self.declare_parameter('recording_directory', '.')
        recording_directory = self.get_parameter('recording_directory')
        self.recording_directory = recording_directory.get_parameter_value().string_value

        self.declare_parameter('recording_name', 'rosbag2')
        recording_name = self.get_parameter('recording_name')
        self.recording_name = recording_name.get_parameter_value().string_value

        self.declare_parameter('prepend_datetime', True)
        prepend_datetime = self.get_parameter('prepend_datetime')
        self.prepend_datetime = prepend_datetime.get_parameter_value().bool_value

        self.declare_parameter('storage', 'mcap')
        storage = self.get_parameter('storage')
        self.storage = storage.get_parameter_value().string_value

        self.declare_parameter('max_cache_size', 100 * 1024 * 1024)
        max_cache_size = self.get_parameter('max_cache_size')
        self.max_cache_size = max_cache_size.get_parameter_value().integer_value

        self.declare_parameter('compression_mode', 'none')
        compression_mode = self.get_parameter('compression_mode')
        self.compression_mode = compression_mode.get_parameter_value().string_value

        self.declare_parameter('enable_services', True)
        enable_services = self.get_parameter('enable_services')
        self.enable_services = enable_services.get_parameter_value().bool_value

        self.process = None
        self.start = None
        self.metadata = {}

        if self.enable_services:
            self.create_service(StartRecording, 'rosbag2/start_recording', self.start_recording)
            self.create_service(StopRecording, 'rosbag2/stop_recording', self.stop_recording)

            self.publisher = self.create_publisher(RecordingInfo, 'rosbag2/recording_info', 10)
            self.create_timer(0.1, self.recording_info)
        else:
            self.metadata['uuid'] = str(uuid.uuid4())
            self.metadata['author'] = 'nvidia'
            self.metadata['title'] = self.recording_name

            if os.path.exists('/etc/hostname'):
                with open('/etc/hostname', 'r') as file:
                    self.metadata['hostname'] = file.read().strip('\n')

            self.rosbag_name = re.sub('[^0-9a-zA-Z-]+', '_', self.recording_name).lower()
            if self.prepend_datetime:
                timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
                self.rosbag_name = timestamp + '_' + self.rosbag_name
            self.output = self.recording_directory + '/' + self.rosbag_name

            args = [
                '--storage', self.storage,
                '--output', self.output,
                '--max-cache-size', str(self.max_cache_size),
                '--compression-mode', self.compression_mode,
            ]
            self.process = subprocess.Popen(['ros2', 'bag', 'record'] + args + self.topics)

    def __del__(self):
        self.stop_recording(None, None)

    def recording_info(self):
        if self.start:
            self.metadata['duration'] = (datetime.now() - self.start).total_seconds()

        msg = RecordingInfo()
        msg.recording = bool(self.process)
        msg.timestamp = self.metadata['timestamp'] if 'timestamp' in self.metadata else 0.0
        msg.duration = self.metadata['duration'] if 'duration' in self.metadata else 0.0
        msg.uuid = self.metadata['uuid'] if 'uuid' in self.metadata else ''
        msg.author = self.metadata['author'] if 'author' in self.metadata else ''
        msg.title = self.metadata['title'] if 'title' in self.metadata else ''
        msg.location = self.metadata['location'] if 'location' in self.metadata else ''
        msg.description = self.metadata['description'] if 'description' in self.metadata else ''
        msg.hostname = self.metadata['hostname'] if 'hostname' in self.metadata else ''
        self.publisher.publish(msg)

    def start_recording(self, request, response):
        if not self.process:
            self.start = datetime.now()

            self.metadata['timestamp'] = self.start.timestamp()
            self.metadata['duration'] = 0.0
            self.metadata['uuid'] = str(uuid.uuid4())
            self.metadata['author'] = request.author if request.author else 'unknown'
            self.metadata['title'] = request.title if request.title else 'rosbag2'
            self.metadata['location'] = request.location if request.location else 'unknown'
            self.metadata['description'] = request.description if request.description else 'none'

            if os.path.exists('/etc/hostname'):
                with open('/etc/hostname', 'r') as file:
                    self.metadata['hostname'] = file.read().strip('\n')

            self.rosbag_name = re.sub('[^0-9a-zA-Z-]+', '_', self.metadata['title']).lower()
            self.rosbag_name = self.start.strftime('%Y-%m-%d_%H-%M-%S') + '_' + self.rosbag_name
            self.output = self.recording_directory + '/' + self.rosbag_name

            args = [
                '--storage', self.storage,
                '--output', self.output,
                '--max-cache-size', str(self.max_cache_size),
                '--compression-mode', self.compression_mode,
            ]
            self.process = subprocess.Popen(['ros2', 'bag', 'record'] + args + self.topics)
        return response

    def stop_recording(self, request, response):
        if self.process:
            self.process.terminate()
            time.sleep(1)

            path = self.output + '/' + self.rosbag_name + '_0.mcap'
            if os.path.exists(path):
                mcap_add_metadata(self.metadata, path)
                for file in self.files:
                    if file:
                        if os.path.exists(file):
                            mcap_add_attachment(file, path)

                if self.enable_services:
                    result = data_validation(self.output)
                    self.get_logger().info('\n' + result.stdout)

            self.process = None
            self.start = None
            self.metadata.clear()
        return response


def main():
    rclpy.init()
    rclpy.spin(Rosbag2Node())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
