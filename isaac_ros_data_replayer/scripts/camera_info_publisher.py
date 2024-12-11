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

from copy import deepcopy
import json
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
import yaml


def json_to_camera_info(json_content):
    msg = CameraInfo()
    msg.height = json_content['height']
    msg.width = json_content['width']
    msg.distortion_model = json_content['distortion_model']
    msg.d = json_content['d']
    msg.k = json_content['k']
    msg.r = json_content['r']
    msg.p = json_content['p']
    msg.binning_x = json_content['binning_x']
    msg.binning_y = json_content['binning_y']
    msg.roi.x_offset = json_content['roi']['x_offset']
    msg.roi.y_offset = json_content['roi']['y_offset']
    msg.roi.height = json_content['roi']['height']
    msg.roi.width = json_content['roi']['width']
    msg.roi.do_rectify = json_content['roi']['do_rectify']
    return msg


def yaml_to_camera_info(yaml_content):

    def as_floats_list(combined_list):
        return [float(element) for element in combined_list]

    msg = CameraInfo()
    msg.width = yaml_content['image_width']
    msg.height = yaml_content['image_height']
    msg.k = as_floats_list(yaml_content['camera_matrix']['data'])
    msg.distortion_model = yaml_content['distortion_model']
    msg.d = as_floats_list(yaml_content['distortion_coefficients']['data'])
    msg.r = as_floats_list(yaml_content['rectification_matrix']['data'])
    msg.p = as_floats_list(yaml_content['projection_matrix']['data'])
    # Yaml file does not contain binning and roi. Setting them to default values(zeros and false)
    return msg


def load_camera_info(camera_info_path):
    camera_info_msg = None
    _, file_extension = os.path.splitext(camera_info_path)
    if file_extension == '.yaml':
        with open(camera_info_path, 'r', encoding='utf8') as fh:
            yaml_content = yaml.safe_load(fh)
            camera_info_msg = yaml_to_camera_info(yaml_content)
    elif file_extension == '.json':
        with open(camera_info_path, 'r', encoding='utf8') as fh:
            json_content = json.load(fh)
            camera_info_msg = json_to_camera_info(json_content)
    else:
        raise ValueError(f'Extension not supported: {file_extension}. Supported: .json, .yaml')
    return camera_info_msg


class CameraInfoPublisher(Node):

    def __init__(self):
        super().__init__('camera_info_publisher')

        self.declare_parameter('output_camera_info_topic', rclpy.Parameter.Type.STRING)
        self.output_camera_info_topic = self.get_parameter(
            'output_camera_info_topic').get_parameter_value().string_value

        self.declare_parameter('input_path', rclpy.Parameter.Type.STRING)
        self.input_path = self.get_parameter('input_path').get_parameter_value().string_value

        self.declare_parameter('input_image_topic', rclpy.Parameter.Type.STRING)
        self.input_image_topic = self.get_parameter(
            'input_image_topic').get_parameter_value().string_value

        self.publisher_ = self.create_publisher(CameraInfo, self.output_camera_info_topic, 10)

        self.image_subscription = self.create_subscription(
            Image,
            self.input_image_topic,
            self.camera_info_publication_callback,
            10)

        self.camera_info_msg = load_camera_info(self.input_path)

    def camera_info_publication_callback(self, image_msg):
        if self.camera_info_msg:
            msg_to_publish = deepcopy(self.camera_info_msg)
            msg_to_publish.header.stamp = image_msg.header.stamp
            self.publisher_.publish(msg_to_publish)
        else:
            self.get_logger().info('Intrinsics not loaded. Not publishing camera_info for the'
                                   f' image message with timestamp {image_msg.header.stamp}')


def main(args=None):
    rclpy.init(args=args)
    camera_info_publisher = CameraInfoPublisher()
    rclpy.spin(camera_info_publisher)
    camera_info_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
