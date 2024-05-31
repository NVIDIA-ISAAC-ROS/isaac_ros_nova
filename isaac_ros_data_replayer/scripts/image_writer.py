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

import cv2
import cv_bridge
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from stereo_msgs.msg import DisparityImage


class ImageWriter(Node):

    def __init__(self):
        super().__init__('image_writer')

        self.declare_parameter('topic', rclpy.Parameter.Type.STRING)
        self.topic = self.get_parameter('topic').get_parameter_value().string_value

        self.declare_parameter('output', rclpy.Parameter.Type.STRING)
        self.output = self.get_parameter('output').get_parameter_value().string_value

        self.declare_parameter('min_disparity', rclpy.Parameter.Type.DOUBLE)
        self.min_disparity = self.get_parameter('min_disparity').get_parameter_value().double_value

        self.declare_parameter('max_disparity', rclpy.Parameter.Type.DOUBLE)
        self.max_disparity = self.get_parameter('max_disparity').get_parameter_value().double_value

        if 'image' in self.topic:
            self.create_subscription(Image, self.topic, self.image_callback, 10)
        elif 'disparity' in self.topic:
            self.create_subscription(DisparityImage, self.topic, self.disparity_callback, 10)
        elif 'depth' in self.topic:
            self.create_subscription(Image, self.topic, self.depth_callback, 10)
        else:
            raise Exception(f"Unsupported topic '{self.topic}'")

        self.directory = self.output + '/' + self.topic.split('/')[0]
        os.makedirs(self.directory, exist_ok=True)

        self.bridge = cv_bridge.CvBridge()
        self.prefix = self.directory + '/' + self.topic.split('/')[0] + '_'
        self.count = 0

    def image_callback(self, msg):
        path = self.prefix + str(self.count).zfill(5) + '.png'
        image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        cv2.imwrite(path, image)
        self.count += 1

    def disparity_callback(self, msg):
        path = self.prefix + str(self.count).zfill(5) + '.png'
        disparity = self.bridge.imgmsg_to_cv2(msg.image)
        disparity = disparity.clip(self.min_disparity, self.max_disparity)
        disparity = (disparity - self.min_disparity) / self.max_disparity * 255
        image = cv2.applyColorMap(disparity.astype(np.uint8), cv2.COLORMAP_TURBO)
        cv2.imwrite(path, image)
        self.count += 1

    def depth_callback(self, msg):
        path = self.prefix + str(self.count).zfill(5) + '.pfm'
        image = self.bridge.imgmsg_to_cv2(msg)
        cv2.imwrite(path, image)
        self.count += 1


def main():
    rclpy.init()
    rclpy.spin(ImageWriter())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
