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
from multiprocessing import Lock, Process
import os
import signal
import subprocess
import time

from camera_info_publisher import load_camera_info
import cv2
import cv_bridge
from launch import LaunchDescription, LaunchService
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
import numpy as np
import rclpy
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from stereo_msgs.msg import DisparityImage


DEFAULT_ENGINE_FILE_PATH = 'isaac_ros_assets/models/dnn_stereo_disparity' \
                         + '/dnn_stereo_disparity_v4.1.0_onnx/ess.engine'


class DataExtraction:

    def __init__(self,
                 rosbag,
                 camera,
                 output,
                 min_disparity=3.0,
                 max_disparity=200.0,
                 threshold=0.0,
                 engine_file_path=DEFAULT_ENGINE_FILE_PATH,
                 mode='depth',
                 encode_mp4=True,
                 image_extension='.png'):
        self.rosbag = rosbag
        self.namespace = camera
        if camera in output:
            self.output = output
        else:
            self.output = output + '/' + camera

        os.makedirs(self.output, exist_ok=True)
        self.min_disparity = min_disparity
        self.max_disparity = max_disparity
        self.threshold = threshold
        self.engine_file_path = engine_file_path
        if mode not in ('raw', 'rectify', 'depth'):
            raise ValueError("'mode' must be 'raw', 'rectify', or 'depth'")
        self.mode = mode
        self.encode_mp4 = encode_mp4
        self.image_extension = image_extension

        self.bridge = cv_bridge.CvBridge()
        self.left_image_msgs = {}
        self.right_image_msgs = {}
        self.left_timestamp_file = open(self.output + '/left_timestamp.txt', 'wt')
        self.right_timestamp_file = open(self.output + '/right_timestamp.txt', 'wt')
        self.image_count = 0
        self.disparity_count = 0
        self.depth_count = 0
        self.lock = Lock()

    def remove_top_n(cache, n):
        # Sort the dictionary by keys
        sorted_dict = dict(sorted(cache.items()))
        # Get the keys to remove
        keys_to_remove = list(sorted_dict.keys())[:n]
        # Remove the first n items
        for key in keys_to_remove:
            del sorted_dict[key]
        return sorted_dict

    def store(msg, cache):
        if msg.header.stamp.sec not in cache:
            cache[msg.header.stamp.sec] = {}
        cache[msg.header.stamp.sec][msg.header.stamp.nanosec] = msg
        # we only cache maximum 5s images
        if len(cache) > 5:
            DataExtraction.remove_top_n(cache, 1)

    def load(msg, cache):
        if (msg.header.stamp.sec in cache and
                msg.header.stamp.nanosec in cache[msg.header.stamp.sec]):
            return cache[msg.header.stamp.sec].pop(msg.header.stamp.nanosec)
        else:
            return None

    def write_image_pair(self, left_image_msg, right_image_msg):
        for value in ('left', 'right'):
            path = self.output + '/' + value + '/' + \
                str(self.image_count).zfill(10) + self.image_extension
            if value == 'left':
                timestamp_file = self.left_timestamp_file
                msg = left_image_msg
            else:
                timestamp_file = self.right_timestamp_file
                msg = right_image_msg
            image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            cv2.imwrite(path, image)
            timestamp_file.write(str(msg.header.stamp.sec * 1000000000 +
                                 msg.header.stamp.nanosec) + '\n')

        self.image_count += 1

    def left_image_callback(self, left_image_msg):
        right_image_msg = DataExtraction.load(left_image_msg, self.right_image_msgs)
        if right_image_msg:
            self.write_image_pair(left_image_msg, right_image_msg)
        else:
            DataExtraction.store(left_image_msg, self.left_image_msgs)

    def right_image_callback(self, right_image_msg):
        left_image_msg = DataExtraction.load(right_image_msg, self.left_image_msgs)
        if left_image_msg:
            self.write_image_pair(left_image_msg, right_image_msg)
        else:
            DataExtraction.store(right_image_msg, self.right_image_msgs)

    def disparity_callback(self, disparity_msg):
        path = self.output + '/disparity/' + str(self.disparity_count).zfill(10) + '.png'
        disparity = self.bridge.imgmsg_to_cv2(disparity_msg.image)
        disparity = disparity.clip(self.min_disparity, self.max_disparity)
        disparity = (disparity - self.min_disparity) / self.max_disparity * 255
        image = cv2.applyColorMap(disparity.astype(np.uint8), cv2.COLORMAP_TURBO)
        cv2.imwrite(path, image)

        with self.lock:
            self.disparity_count += 1

    def depth_callback(self, depth_msg):
        path = self.output + '/depth/' + str(self.depth_count).zfill(10) + '.pfm'
        image = self.bridge.imgmsg_to_cv2(depth_msg)
        cv2.imwrite(path, image)

        with self.lock:
            self.depth_count += 1

    def decode(self, camera_info_topic='/camera_info'):
        nodes = []

        topic = camera_info_topic if self.mode == 'raw' else '/camera_info_resized'

        left_camera_info_writer = Node(
            name='left_camera_info_writer',
            package='isaac_ros_data_replayer',
            executable='camera_info_writer.py',
            namespace=self.namespace,
            parameters=[{
                'topic': 'left' + topic,
                'output': self.output,
            }],
        )

        right_camera_info_writer = Node(
            name='right_camera_info_writer',
            package='isaac_ros_data_replayer',
            executable='camera_info_writer.py',
            namespace=self.namespace,
            parameters=[{
                'topic': 'right' + topic,
                'output': self.output,
            }],
        )

        nodes.append(ComposableNode(
            name='left_decoder',
            package='isaac_ros_h264_decoder',
            plugin='nvidia::isaac_ros::h264_decoder::DecoderNode',
            namespace=self.namespace,
            remappings=[
                ('image_compressed', 'left/image_compressed'),
                ('image_uncompressed', 'left/image_raw'),
            ],
        ))

        nodes.append(ComposableNode(
            name='right_decoder',
            package='isaac_ros_h264_decoder',
            plugin='nvidia::isaac_ros::h264_decoder::DecoderNode',
            namespace=self.namespace,
            remappings=[
                ('image_compressed', 'right/image_compressed'),
                ('image_uncompressed', 'right/image_raw'),
            ],
        ))

        if self.mode in ('rectify', 'depth'):
            output_width = 960 if self.mode == 'depth' else 1920
            output_height = 576 if self.mode == 'depth' else 1200

            nodes.append(ComposableNode(
                name='left_rectify',
                package='isaac_ros_image_proc',
                plugin='nvidia::isaac_ros::image_proc::RectifyNode',
                namespace=self.namespace,
                remappings=[
                    ('image_raw', 'left/image_raw'),
                    ('camera_info', 'left' + camera_info_topic),
                    ('image_rect', 'left/image_rect'),
                    ('camera_info_rect', 'left/camera_info_rect'),
                ],
                parameters=[{
                    'output_width': 1920,
                    'output_height': 1200,
                }],
            ))

            nodes.append(ComposableNode(
                name='right_rectify',
                package='isaac_ros_image_proc',
                plugin='nvidia::isaac_ros::image_proc::RectifyNode',
                namespace=self.namespace,
                remappings=[
                    ('image_raw', 'right/image_raw'),
                    ('camera_info', 'right' + camera_info_topic),
                    ('image_rect', 'right/image_rect'),
                    ('camera_info_rect', 'right/camera_info_rect'),
                ],
                parameters=[{
                    'output_width': 1920,
                    'output_height': 1200,
                }],
            ))

            nodes.append(ComposableNode(
                name='left_resize',
                package='isaac_ros_image_proc',
                plugin='nvidia::isaac_ros::image_proc::ResizeNode',
                namespace=self.namespace,
                remappings=[
                    ('image', 'left/image_rect'),
                    ('camera_info', 'left/camera_info_rect'),
                    ('resize/image', 'left/image_resized'),
                    ('resize/camera_info', 'left/camera_info_resized'),
                ],
                parameters=[{
                    'output_width': output_width,
                    'output_height': output_height,
                }],
            ))

            nodes.append(ComposableNode(
                name='right_resize',
                package='isaac_ros_image_proc',
                plugin='nvidia::isaac_ros::image_proc::ResizeNode',
                namespace=self.namespace,
                remappings=[
                    ('image', 'right/image_rect'),
                    ('camera_info', 'right/camera_info_rect'),
                    ('resize/image', 'right/image_resized'),
                    ('resize/camera_info', 'right/camera_info_resized'),
                ],
                parameters=[{
                    'output_width': output_width,
                    'output_height': output_height,
                }],
            ))

        container = ComposableNodeContainer(
            name='rectify_node',
            package='rclcpp_components',
            executable='component_container_mt',
            namespace=self.namespace,
            composable_node_descriptions=nodes,
        )

        launch = LaunchService()
        launch.include_launch_description(LaunchDescription([
            left_camera_info_writer,
            right_camera_info_writer,
            container,
        ]))

        launch.run()
        launch.shutdown()

    def ess(self):
        ess = ComposableNode(
            name='ess',
            package='isaac_ros_ess',
            plugin='nvidia::isaac_ros::dnn_stereo_depth::ESSDisparityNode',
            namespace=self.namespace,
            remappings=[
                ('left/camera_info', 'left/camera_info_rect'),
                ('right/camera_info', 'right/camera_info_rect'),
            ],
            parameters=[{
                'engine_file_path': self.engine_file_path,
                'threshold': self.threshold,
            }],
        )

        disparity_to_depth = ComposableNode(
            name='disparity_to_depth',
            package='isaac_ros_stereo_image_proc',
            plugin='nvidia::isaac_ros::stereo_image_proc::DisparityToDepthNode',
            namespace=self.namespace,
        )

        container = ComposableNodeContainer(
            name='ess_node',
            package='rclcpp_components',
            executable='component_container_mt',
            namespace=self.namespace,
            composable_node_descriptions=[
                ess,
                disparity_to_depth,
            ],
        )

        launch = LaunchService()
        launch.include_launch_description(LaunchDescription([
            container,
        ]))

        launch.run()
        launch.shutdown()

    def png_to_mp4(self, stream):
        ffmpeg = [
            'ffmpeg', '-nostdin', '-framerate', '30', '-pattern_type', 'glob', '-i',
            self.output + '/' + stream + '/*.png', '-c:v', 'libx264', '-r', '30',
            self.output + '/' + self.namespace + '_' + stream + '.mp4', '-y'
        ]
        return subprocess.run(ffmpeg, check=True)

    def extract_data(self):
        reader = SequentialReader()
        reader.open(
            StorageOptions(uri=self.rosbag, storage_id='mcap'),
            ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr'),
        )

        # use 'camerainfo' to maintain compatibility with older recordings
        camera_info_topic = '/camera_info'
        for topic_type in reader.get_all_topics_and_types():
            if self.namespace in topic_type.name and topic_type.name.endswith('/camerainfo'):
                camera_info_topic = '/camerainfo'
                break

        topics = {
            'left_camera_info':       '/' + self.namespace + '/left' + camera_info_topic,
            'right_camera_info':      '/' + self.namespace + '/right' + camera_info_topic,
            'left_camera_info_rect':  '/' + self.namespace + '/left/camera_info_rect',
            'right_camera_info_rect': '/' + self.namespace + '/right/camera_info_rect',
            'left_image_compressed':  '/' + self.namespace + '/left/image_compressed',
            'right_image_compressed': '/' + self.namespace + '/right/image_compressed',
            'left_image_raw':         '/' + self.namespace + '/left/image_raw',
            'right_image_raw':        '/' + self.namespace + '/right/image_raw',
            'left_image_rect':        '/' + self.namespace + '/left/image_rect',
            'right_image_rect':       '/' + self.namespace + '/right/image_rect',
            'left_image_resized':     '/' + self.namespace + '/left/image_resized',
            'right_image_resized':    '/' + self.namespace + '/right/image_resized',
            'disparity':              '/' + self.namespace + '/disparity',
            'depth':                  '/' + self.namespace + '/depth',
        }

        rclpy.init()

        node = rclpy.create_node('data_extraction', namespace=self.namespace)

        left_camera_info = node.create_publisher(
            CameraInfo, topics['left_camera_info'], 10)
        right_camera_info = node.create_publisher(
            CameraInfo, topics['right_camera_info'], 10)
        left_image_compressed = node.create_publisher(
            CompressedImage, topics['left_image_compressed'], 10)
        right_image_compressed = node.create_publisher(
            CompressedImage, topics['right_image_compressed'], 10)

        image_topic = 'image_raw' if self.mode == 'raw' else 'image_resized'
        left_image = node.create_subscription(
            Image, topics['left_' + image_topic], self.left_image_callback, 10)
        right_image = node.create_subscription(
            Image, topics['right_' + image_topic], self.right_image_callback, 10)

        left_camera_info_msg = None
        right_camera_info_msg = None

        while reader.has_next():
            topic, data, timestamp = reader.read_next()
            if left_camera_info_msg is not None and right_camera_info_msg is not None:
                break
            elif (topic == topics['left_camera_info'] and left_camera_info_msg is None):
                left_camera_info_msg = deserialize_message(data, CameraInfo)
            elif (topic == topics['right_camera_info'] and right_camera_info_msg is None):
                right_camera_info_msg = deserialize_message(data, CameraInfo)

        if left_camera_info_msg is None:
            raise ValueError('No camera info found for left camera')
        elif right_camera_info_msg is None:
            raise ValueError('No camera info found for right camera')

        os.makedirs(self.output + '/left', exist_ok=True)
        os.makedirs(self.output + '/right', exist_ok=True)

        decode = Process(target=self.decode, args=(camera_info_topic,))
        decode.start()
        time.sleep(5)

        while reader.has_next():
            topic, data, timestamp = reader.read_next()
            if topic == topics['left_image_compressed']:
                left_image_compressed_msg = deserialize_message(data, CompressedImage)
                left_camera_info_msg.header.stamp = left_image_compressed_msg.header.stamp
                left_camera_info.publish(left_camera_info_msg)
                left_image_compressed.publish(left_image_compressed_msg)
                rclpy.spin_once(node, timeout_sec=1.0)
            elif topic == topics['right_image_compressed']:
                right_image_compressed_msg = deserialize_message(data, CompressedImage)
                right_camera_info_msg.header.stamp = right_image_compressed_msg.header.stamp
                right_camera_info.publish(right_camera_info_msg)
                right_image_compressed.publish(right_image_compressed_msg)
                rclpy.spin_once(node, timeout_sec=1.0)

        os.kill(decode.pid, signal.SIGINT)
        decode.join()

        node.destroy_subscription(left_image)
        node.destroy_subscription(right_image)

        node.destroy_publisher(right_image_compressed)
        node.destroy_publisher(left_image_compressed)
        node.destroy_publisher(right_camera_info)
        node.destroy_publisher(left_camera_info)

        if self.encode_mp4:
            self.png_to_mp4('left')
            self.png_to_mp4('right')

        if self.mode == 'depth':
            left_camera_info_msg = load_camera_info(
                self.output + '/left_camera_info_resized.json')
            right_camera_info_msg = load_camera_info(
                self.output + '/right_camera_info_resized.json')

            left_camera_info_rect = node.create_publisher(
                CameraInfo, topics['left_camera_info_rect'], 10)
            right_camera_info_rect = node.create_publisher(
                CameraInfo, topics['right_camera_info_rect'], 10)
            left_image_rect = node.create_publisher(
                Image, topics['left_image_rect'], 10)
            right_image_rect = node.create_publisher(
                Image, topics['right_image_rect'], 10)

            disparity = node.create_subscription(
                DisparityImage, topics['disparity'], self.disparity_callback, 10)
            depth = node.create_subscription(
                Image, topics['depth'], self.depth_callback, 10)

            os.makedirs(self.output + '/disparity', exist_ok=True)
            os.makedirs(self.output + '/depth', exist_ok=True)

            ess = Process(target=self.ess)
            ess.start()
            time.sleep(5)

            for i in range(self.image_count):
                for value in ('left', 'right'):
                    path = self.output + '/' + value + '/' + str(i).zfill(10) + '.png'
                    image = cv2.imread(path)
                    msg = self.bridge.cv2_to_imgmsg(image, 'rgb8')

                    if value == 'left':
                        left_camera_info_rect.publish(left_camera_info_msg)
                        left_image_rect.publish(msg)
                    elif value == 'right':
                        right_camera_info_rect.publish(right_camera_info_msg)
                        right_image_rect.publish(msg)

                spin_count = i + 1

                with self.lock:
                    done = self.disparity_count == spin_count and self.depth_count == spin_count

                while not done:
                    rclpy.spin_once(node)

                    with self.lock:
                        done = (self.disparity_count == spin_count and
                                self.depth_count == spin_count)

            os.kill(ess.pid, signal.SIGINT)
            ess.join()

            node.destroy_subscription(depth)
            node.destroy_subscription(disparity)

            node.destroy_publisher(right_image_rect)
            node.destroy_publisher(left_image_rect)
            node.destroy_publisher(right_camera_info_rect)
            node.destroy_publisher(left_camera_info_rect)

            if self.encode_mp4:
                self.png_to_mp4('disparity')

        node.destroy_node()

        rclpy.shutdown()


def parse_args():
    parser = ArgumentParser(prog='data_extraction.py')
    parser.add_argument('--rosbag',
                        help='path to rosbag',
                        type=str,
                        required=True)
    parser.add_argument('--camera',
                        help='camera to extract',
                        type=str,
                        required=True)
    parser.add_argument('--output',
                        help='output directory',
                        type=str,
                        required=True)
    parser.add_argument('--min_disparity',
                        help='minimum disparity value',
                        type=float,
                        default=3.0)
    parser.add_argument('--max_disparity',
                        help='maximum disparity value',
                        type=float,
                        default=200.0)
    parser.add_argument('--threshold',
                        help='ESS threshold',
                        type=float,
                        default=0.0)
    parser.add_argument('--engine_file_path',
                        help='ESS engine file path',
                        type=str,
                        default=DEFAULT_ENGINE_FILE_PATH)
    parser.add_argument('--encode_mp4',
                        help='Whether or not encode mp4',
                        action='store_true')
    parser.add_argument('--mode',
                        help="image extraction mode ('raw', 'rectify', or 'depth')",
                        type=str,
                        default='depth')
    parser.add_argument('--image_extension',
                        help='The file extension of the image to save',
                        type=str,
                        default='.png')
    return parser.parse_args()


def main():
    args = parse_args()
    de = DataExtraction(
        rosbag=args.rosbag,
        camera=args.camera,
        output=args.output,
        min_disparity=args.min_disparity,
        max_disparity=args.max_disparity,
        threshold=args.threshold,
        engine_file_path=args.engine_file_path,
        mode=args.mode,
        encode_mp4=args.encode_mp4,
        image_extension=args.image_extension)
    de.extract_data()


if __name__ == '__main__':
    main()
