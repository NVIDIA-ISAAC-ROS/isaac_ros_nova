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

import json
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo


class CameraInfoWriter(Node):

    def __init__(self):
        super().__init__('camera_info_writer')

        self.declare_parameter('topic', rclpy.Parameter.Type.STRING)
        self.topic = self.get_parameter('topic').get_parameter_value().string_value

        self.declare_parameter('output', rclpy.Parameter.Type.STRING)
        self.output = self.get_parameter('output').get_parameter_value().string_value

        self.create_subscription(CameraInfo, self.topic, self.callback, 10)

        os.makedirs(self.output, exist_ok=True)
        path = self.output + '/' + self.topic.replace('/', '_') + '.json'
        self.file = open(path, 'w')

    def callback(self, msg):
        if not self.file.closed:
            intrinsics = {
                'height': msg.height,
                'width': msg.width,
                'distortion_model': msg.distortion_model,
                'd': list(msg.d),
                'k': list(msg.k),
                'r': list(msg.r),
                'p': list(msg.p),
                'binning_x': msg.binning_x,
                'binning_y': msg.binning_y,
                'roi': {
                    'x_offset': msg.roi.x_offset,
                    'y_offset': msg.roi.y_offset,
                    'height': msg.roi.height,
                    'width': msg.roi.width,
                    'do_rectify': msg.roi.do_rectify,
                },
            }
            self.file.write(json.dumps(intrinsics, indent=2))
            self.file.close()


def main():
    rclpy.init()
    rclpy.spin(CameraInfoWriter())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
