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

from argparse import ArgumentParser
import subprocess
from tempfile import TemporaryDirectory
from typing import List

import cv2
import cv_bridge
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from sensor_msgs.msg import Image


def ffmpeg(args: List[str]):
    return subprocess.run(['ffmpeg', '-nostdin'] + args, check=True)


def convert(rosbag: str, camera: str, output: str):
    if not camera.startswith('/'):
        camera = '/' + camera

    directory = TemporaryDirectory()

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=rosbag, storage_id='mcap'),
        ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr'),
    )

    if camera.endswith('image_compressed'):
        filename = camera.replace('/', '', 1).replace('/', '_')
        filename = filename.replace('_image_compressed', '.h264')
        file = open(directory.name + '/' + filename, 'wb')

        while reader.has_next():
            topic, data, timestamp = reader.read_next()
            if camera == topic:
                file.write(data)

        ffmpeg(['-i', file.name, '-c', 'copy', output, '-y'])

        file.close()
    else:
        bridge = cv_bridge.CvBridge()
        count = 0

        while reader.has_next():
            topic, data, timestamp = reader.read_next()
            if camera == topic:
                count += 1
                msg = deserialize_message(data, Image)
                image = bridge.imgmsg_to_cv2(msg, 'bgr8')
                path = directory.name + '/' + str(count).zfill(5) + '.png'
                cv2.imwrite(path, image)

        ffmpeg(['-framerate', '30', '-pattern_type', 'glob', '-i',
                directory.name + '/*.png', '-c:v', 'libx264', '-r', '30', output, '-y'])

    directory.cleanup()


def main():
    parser = ArgumentParser(
        prog='camera_converter.py',
        description='converts a camera stream in a rosbag to a video file',
    )
    parser.add_argument('-i', '--input',
                        help='input rosbag path',
                        type=str,
                        required=True)
    parser.add_argument('-t', '--topic',
                        help="camera topic to convert i.e. 'front_stereo_camera/left/image_raw'",
                        type=str,
                        required=True)
    parser.add_argument('-o', '--output',
                        help="output video path i.e '/tmp/front_stereo_camera.mp4",
                        type=str,
                        required=True)
    args = parser.parse_args()

    convert(rosbag=args.input, camera=args.topic, output=args.output)


if __name__ == '__main__':
    main()
