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

import cv_bridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image


class JpegEncoderNode(Node):

    def __init__(self):
        super().__init__('jpeg_encoder')

        self.create_subscription(Image, 'image_raw', self.callback, 10)
        self.publisher = self.create_publisher(CompressedImage, 'image_compressed', 10)

        self.bridge = cv_bridge.CvBridge()

    def callback(self, image_msg):
        image = self.bridge.imgmsg_to_cv2(image_msg, 'bgr8')
        compressed_image_msg = self.bridge.cv2_to_compressed_imgmsg(image)
        self.publisher.publish(compressed_image_msg)

def main():
    rclpy.init()
    rclpy.spin(JpegEncoderNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
