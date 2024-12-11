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

import os
import shutil

from isaac_ros_nova_recorder.msg import DirectoryInfo, RosbagInfo
from isaac_ros_nova_recorder.srv import DeleteRosbag
import rclpy
from rclpy.node import Node


class RosbagManagerNode(Node):

    def __init__(self):
        super().__init__('rosbag_manager')

        self.declare_parameter('directory', '.')
        self.directory = self.get_parameter('directory').get_parameter_value().string_value

        self.create_service(DeleteRosbag, 'delete_rosbag', self.delete_rosbag)

        self.publisher = self.create_publisher(DirectoryInfo, 'directory_info', 10)
        self.create_timer(0.1, self.directory_info)

    def delete_rosbag(self, request, response):
        path = os.path.join(self.directory, request.rosbag)
        if os.path.exists(path) and os.path.isdir(path):
            shutil.rmtree(path)
            response.success = True
        else:
            response.success = False
        return response

    def directory_info(self):
        msg = DirectoryInfo()
        statvfs = os.statvfs(self.directory)
        msg.path = self.directory
        msg.size = statvfs.f_bsize * statvfs.f_blocks
        msg.free = statvfs.f_bsize * statvfs.f_bfree
        msg.used = float(msg.size - msg.free) / float(msg.size)
        msg.rosbags = []
        for path, directories, files in os.walk(self.directory):
            if path == self.directory:
                continue
            info = RosbagInfo()
            info.rosbag = os.path.basename(path)
            info.size = 0
            for file in files:
                if file == 'metadata.yaml' or file.endswith('.mcap'):
                    info.size += os.path.getsize(os.path.join(path, file))
            info.time = os.path.getctime(path)
            msg.rosbags.append(info)

        self.publisher.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(RosbagManagerNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
