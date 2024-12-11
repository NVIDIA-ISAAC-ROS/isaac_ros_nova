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
import yaml

SUPPORTED_EXTENSIONS = ('.json', '.yaml')


def camera_info_to_yaml_dictionary(msg):
    intrinsics = {
        'image_height': msg.height,
        'image_width': msg.width,
        'distortion_model': msg.distortion_model,
        'distortion_coefficients': {
            'rows': 1,
            'cols': len(msg.d),
            'data': msg.d.tolist(),
        },
        'camera_matrix': {
            'rows': 3,
            'cols': 3,
            'data': msg.k.tolist(),
        },
        'rectification_matrix': {
            'rows': 3,
            'cols': 3,
            'data': msg.r.tolist(),
        },
        'projection_matrix': {
            'rows': 3,
            'cols': 4,
            'data': msg.p.tolist(),
        },
    }
    return intrinsics


def camera_info_to_json_dictionary(msg):
    intrinsics = {
        'height': msg.height,
        'width': msg.width,
        'distortion_model': msg.distortion_model,
        'd': msg.d.tolist(),
        'k': msg.k.tolist(),
        'r': msg.r.tolist(),
        'p': msg.p.tolist(),
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
    return intrinsics


def write_camera_info(msg, camera_info_path, json_indent=2):
    _, file_extension = os.path.splitext(camera_info_path)
    if file_extension == '.yaml':
        yaml_dictionary = camera_info_to_yaml_dictionary(msg)
        with open(camera_info_path, 'w', encoding='utf8') as fh:
            yaml.safe_dump(yaml_dictionary, fh)
    elif file_extension == '.json':
        json_content = json.dumps(camera_info_to_json_dictionary(msg), indent=json_indent)
        with open(camera_info_path, 'w', encoding='utf8') as fh:
            fh.write(json_content)
    else:
        raise ValueError(f'Extension not supported: {file_extension}. '
                         f'Supported: {SUPPORTED_EXTENSIONS}')


class CameraInfoWriter(Node):

    def __init__(self):
        super().__init__('camera_info_writer')

        self.declare_parameter('topic', rclpy.Parameter.Type.STRING)
        self.topic = self.get_parameter('topic').get_parameter_value().string_value

        self.declare_parameter('output', rclpy.Parameter.Type.STRING)
        self.output = self.get_parameter('output').get_parameter_value().string_value

        self.declare_parameter('extension', '.json')
        extension = self.get_parameter('extension').get_parameter_value().string_value
        if extension not in SUPPORTED_EXTENSIONS:
            raise ValueError(f'Extension not supported: {extension}. '
                             f'Supported: {SUPPORTED_EXTENSIONS}')

        self.create_subscription(CameraInfo, self.topic, self.callback, 10)

        # Make directory and check if we have permissions to write in the output directory
        os.makedirs(self.output, exist_ok=True)
        self.file_path = self.output + '/' + self.topic.replace('/', '_') + extension
        self.is_file_written = False

    def callback(self, msg):
        if not self.is_file_written:
            write_camera_info(msg, self.file_path)
            self.is_file_written = True


def main():
    rclpy.init()
    rclpy.spin(CameraInfoWriter())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
