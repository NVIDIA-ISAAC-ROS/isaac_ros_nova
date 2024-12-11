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

from collections import deque
from enum import Enum
from datetime import datetime
import os
import re
import shutil
import subprocess
import time

from isaac_ros_data_recorder.srv import Event
import rclpy
from rclpy.node import Node


class EventRecorderNode(Node):

    class State(Enum):
        ERROR = -1,
        UNKNOWN = 0,
        IDLE = 1,
        LOOK_BACK = 2,
        EVENT = 3,
        LOOK_AHEAD = 4,

    def reindex(rosbag, stdout=subprocess.DEVNULL):
        return subprocess.run(['ros2', 'bag', 'reindex', rosbag], stdout=stdout, check=True)

    def __init__(self):
        super().__init__('event_recorder')

        self.process = None

        self.declare_parameter('topics', ['--all'])
        topics = self.get_parameter('topics')
        self.topics = topics.get_parameter_value().string_array_value

        self.declare_parameter('recording_directory', '.')
        recording_directory = self.get_parameter('recording_directory')
        self.recording_directory = recording_directory.get_parameter_value().string_value

        self.declare_parameter('recording_name', 'rosbag2')
        recording_name = self.get_parameter('recording_name')
        self.recording_name = recording_name.get_parameter_value().string_value

        self.declare_parameter('storage', 'mcap')
        storage = self.get_parameter('storage')
        self.storage = storage.get_parameter_value().string_value

        self.declare_parameter('max_bag_duration', 60)
        max_bag_duration = self.get_parameter('max_bag_duration')
        self.max_bag_duration = max_bag_duration.get_parameter_value().integer_value

        self.declare_parameter('max_cache_size', 100 * 1024 * 1024)
        max_cache_size = self.get_parameter('max_cache_size')
        self.max_cache_size = max_cache_size.get_parameter_value().integer_value

        self.declare_parameter('compression_mode', 'none')
        compression_mode = self.get_parameter('compression_mode')
        self.compression_mode = compression_mode.get_parameter_value().string_value

        self.declare_parameter('look_back_window', 60)
        look_back_window = self.get_parameter('look_back_window')
        self.look_back_window = look_back_window.get_parameter_value().integer_value

        self.declare_parameter('look_ahead_window', 60)
        look_ahead_window = self.get_parameter('look_ahead_window')
        self.look_ahead_window = look_ahead_window.get_parameter_value().integer_value

        self.create_service(Event, 'event_start', self.event_start)
        self.create_service(Event, 'event_end', self.event_end)

        # avoid cases where max_bag_duration > look_back_window
        # since we cannot slice recordings in real-time
        if self.max_bag_duration > self.look_back_window > 0:
            self.get_logger().warning(f"max_bag_duration ({self.max_bag_duration}s) cannot be "
                                      f"greater than look_back_window ({self.look_back_window}s)")
            self.get_logger().info(f'Reducing max_bag_duration to {self.look_back_window}s')
            self.max_bag_duration = self.look_back_window

        self.create_timer(1, self.tick)

        if self.look_back_window > 0:
            self.start_recording()
            self.state = EventRecorderNode.State.LOOK_BACK
            self.look_back_buffer = deque()
        else:
            self.state = EventRecorderNode.State.IDLE

    def tick(self):
        if self.state == EventRecorderNode.State.IDLE:
            pass
        elif self.state == EventRecorderNode.State.LOOK_BACK:
            # add new files to look-back buffer
            for path, directories, files in os.walk(self.output):
                if path == self.output:
                    for file in files:
                        if file not in self.look_back_buffer and file.endswith('.mcap'):
                            self.look_back_buffer.append(file)

            # remove stale files from look-back buffer
            # subtract one to account for the file that is currently being recorded
            while ((len(self.look_back_buffer) - 1) * self.max_bag_duration) > self.look_back_window:
                os.remove(os.path.join(self.output, self.look_back_buffer[0]))
                self.look_back_buffer.popleft()
        elif self.state == EventRecorderNode.State.EVENT:
            pass
        elif self.state == EventRecorderNode.State.LOOK_AHEAD:
            # check if look-ahead window expired
            duration = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            if duration > self.look_ahead_window:
                self.stop_recording()
                if self.look_back_window > 0:
                    self.start_recording()
                    self.state = EventRecorderNode.State.LOOK_BACK
                    self.look_back_buffer = deque()
                else:
                    self.state = EventRecorderNode.State.IDLE
                self.get_logger().info('Event recording stopped')
        else:
            self.get_logger().error(f'Unknown state {self.state}')

    def event_start(self, request, response):
        if self.state == EventRecorderNode.State.IDLE or EventRecorderNode.State.LOOK_BACK:
            if self.state == EventRecorderNode.State.IDLE:
                self.start_recording()
            self.state = EventRecorderNode.State.EVENT
            response.success = True
            self.get_logger().info('Event recording started')
        else:
            response.success = False
        return response

    def event_end(self, request, response):
        if self.state == EventRecorderNode.State.EVENT:
            self.state = EventRecorderNode.State.LOOK_AHEAD
            self.start_time = self.get_clock().now()
            response.success = True
        else:
            response.success = False
        return response

    def start_recording(self):
        rosbag_name = re.sub('[^0-9a-zA-Z-]+', '_', self.recording_name).lower()
        rosbag_name = datetime.now().strftime('%Y-%m-%d_%H-%M-%S') + '_' + rosbag_name
        self.output = os.path.join('/tmp', rosbag_name)
        args = [
            '--output', self.output,
            '--storage', self.storage,
            '--max-bag-duration', str(self.max_bag_duration),
            '--max-cache-size', str(self.max_cache_size),
            '--compression-mode', self.compression_mode,
        ]
        self.process = subprocess.Popen(['ros2', 'bag', 'record'] + args + self.topics)

    def stop_recording(self):
        if self.process:
            self.process.terminate()
            time.sleep(1)
            EventRecorderNode.reindex(self.output)
            shutil.move(self.output, self.recording_directory)
            self.process = None

def main():
    rclpy.init()
    rclpy.spin(EventRecorderNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
