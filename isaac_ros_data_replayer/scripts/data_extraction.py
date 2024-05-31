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
import json
from multiprocessing import Lock, Process
import os
import signal
import subprocess
import time

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


ISAAC_ROS_DNN_STEREO_DEPTH = '/workspaces/isaac_ros-dev/ros_ws/src/isaac_ros_dnn_stereo_depth'
DEFAULT_ENGINE_FILE_PATH = ISAAC_ROS_DNN_STEREO_DEPTH + '/resources/ess.engine'


def json_to_camera_info(intrinsics):
    msg = CameraInfo()
    msg.height = intrinsics['height']
    msg.width = intrinsics['width']
    msg.distortion_model = intrinsics['distortion_model']
    msg.d = intrinsics['d']
    msg.k = intrinsics['k']
    msg.r = intrinsics['r']
    msg.p = intrinsics['p']
    msg.binning_x = intrinsics['binning_x']
    msg.binning_y = intrinsics['binning_y']
    msg.roi.x_offset = intrinsics['roi']['x_offset']
    msg.roi.y_offset = intrinsics['roi']['y_offset']
    msg.roi.height = intrinsics['roi']['height']
    msg.roi.width = intrinsics['roi']['width']
    msg.roi.do_rectify = intrinsics['roi']['do_rectify']
    return msg


class DataExtraction:

    def __init__(self,
                 rosbag,
                 camera,
                 output,
                 output_width=960,
                 output_height=576,
                 min_disparity=3.0,
                 max_disparity=200.0,
                 threshold=0.0,
                 engine_file_path=DEFAULT_ENGINE_FILE_PATH):
        self.rosbag = rosbag
        self.namespace = camera
        if camera in output:
            self.output = output
        else:
            self.output = output + '/' + camera

        self.output_width = 960
        self.output_height = 576
        self.min_disparity = min_disparity
        self.max_disparity = max_disparity
        self.threshold = threshold
        self.engine_file_path = engine_file_path

        self.bridge = cv_bridge.CvBridge()
        self.left_image_msgs = {}
        self.right_image_msgs = {}
        self.image_count = 0
        self.disparity_count = 0
        self.depth_count = 0
        self.lock = Lock()

    def store(msg, cache):
        if msg.header.stamp.sec not in cache:
            cache[msg.header.stamp.sec] = {}
        cache[msg.header.stamp.sec][msg.header.stamp.nanosec] = msg

    def load(msg, cache):
        if (msg.header.stamp.sec in cache and
                msg.header.stamp.nanosec in cache[msg.header.stamp.sec]):
            return cache[msg.header.stamp.sec][msg.header.stamp.nanosec]
        else:
            return None

    def write_image_pair(self, left_image_msg, right_image_msg):
        for value in ('left', 'right'):
            path = self.output + '/' + value + '/' + str(self.image_count).zfill(5) + '.png'
            msg = left_image_msg if value == 'left' else right_image_msg
            image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            cv2.imwrite(path, image)

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
        path = self.output + '/disparity/' + str(self.disparity_count).zfill(5) + '.png'
        disparity = self.bridge.imgmsg_to_cv2(disparity_msg.image)
        disparity = disparity.clip(self.min_disparity, self.max_disparity)
        disparity = (disparity - self.min_disparity) / self.max_disparity * 255
        image = cv2.applyColorMap(disparity.astype(np.uint8), cv2.COLORMAP_TURBO)
        cv2.imwrite(path, image)

        with self.lock:
            self.disparity_count += 1

    def depth_callback(self, depth_msg):
        path = self.output + '/depth/' + str(self.depth_count).zfill(5) + '.pfm'
        image = self.bridge.imgmsg_to_cv2(depth_msg)
        cv2.imwrite(path, image)

        with self.lock:
            self.depth_count += 1

    def rectify(self, camera_info_topic='/camera_info'):
        left_camera_info_writer = Node(
            name='left_camera_info_writer',
            package='isaac_ros_data_replayer',
            executable='camera_info_writer.py',
            namespace=self.namespace,
            parameters=[{
                'topic': 'left/camera_info_resized',
                'output': self.output,
            }],
        )

        right_camera_info_writer = Node(
            name='right_camera_info_writer',
            package='isaac_ros_data_replayer',
            executable='camera_info_writer.py',
            namespace=self.namespace,
            parameters=[{
                'topic': 'right/camera_info_resized',
                'output': self.output,
            }],
        )

        left_decoder = ComposableNode(
            name='left_decoder',
            package='isaac_ros_h264_decoder',
            plugin='nvidia::isaac_ros::h264_decoder::DecoderNode',
            namespace=self.namespace,
            remappings=[
                ('image_compressed', 'left/image_compressed'),
                ('image_uncompressed', 'left/image_raw'),
            ],
        )

        right_decoder = ComposableNode(
            name='right_decoder',
            package='isaac_ros_h264_decoder',
            plugin='nvidia::isaac_ros::h264_decoder::DecoderNode',
            namespace=self.namespace,
            remappings=[
                ('image_compressed', 'right/image_compressed'),
                ('image_uncompressed', 'right/image_raw'),
            ],
        )

        left_rectify = ComposableNode(
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
        )

        right_rectify = ComposableNode(
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
        )

        left_resize = ComposableNode(
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
                'output_width': self.output_width,
                'output_height': self.output_height,
            }],
        )

        right_resize = ComposableNode(
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
                'output_width': self.output_width,
                'output_height': self.output_height,
            }],
        )

        container = ComposableNodeContainer(
            name='rectify_node',
            package='rclcpp_components',
            executable='component_container_mt',
            namespace=self.namespace,
            composable_node_descriptions=[
                left_decoder,
                right_decoder,
                left_rectify,
                right_rectify,
                left_resize,
                right_resize,
            ],
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

        left_image_resized = node.create_subscription(
            Image, topics['left_image_resized'], self.left_image_callback, 10)
        right_image_resized = node.create_subscription(
            Image, topics['right_image_resized'], self.right_image_callback, 10)

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

        rectify = Process(target=self.rectify, args=(camera_info_topic,))
        rectify.start()
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

        os.kill(rectify.pid, signal.SIGINT)
        rectify.join()

        node.destroy_subscription(right_image_resized)
        node.destroy_subscription(left_image_resized)

        node.destroy_publisher(right_image_compressed)
        node.destroy_publisher(left_image_compressed)
        node.destroy_publisher(right_camera_info)
        node.destroy_publisher(left_camera_info)

        self.png_to_mp4('left')
        self.png_to_mp4('right')

        left_camera_info_json = open(self.output + '/left_camera_info_resized.json', 'r')
        left_camera_info_msg = json_to_camera_info(json.load(left_camera_info_json))
        left_camera_info_json.close()

        right_camera_info_json = open(self.output + '/right_camera_info_resized.json', 'r')
        right_camera_info_msg = json_to_camera_info(json.load(right_camera_info_json))
        right_camera_info_json.close()

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
                path = self.output + '/' + value + '/' + str(i).zfill(5) + '.png'
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
                    done = self.disparity_count == spin_count and self.depth_count == spin_count

        os.kill(ess.pid, signal.SIGINT)
        ess.join()

        node.destroy_subscription(depth)
        node.destroy_subscription(disparity)

        node.destroy_publisher(right_image_rect)
        node.destroy_publisher(left_image_rect)
        node.destroy_publisher(right_camera_info_rect)
        node.destroy_publisher(left_camera_info_rect)

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
    parser.add_argument('--output_width',
                        help='output width',
                        type=int,
                        default=960)
    parser.add_argument('--output_height',
                        help='output height',
                        type=int,
                        default=576)
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
    return parser.parse_args()


def main():
    args = parse_args()
    de = DataExtraction(
        rosbag=args.rosbag,
        camera=args.camera,
        output=args.output,
        output_width=args.output_width,
        output_height=args.output_height,
        min_disparity=args.min_disparity,
        max_disparity=args.max_disparity,
        threshold=args.threshold,
        engine_file_path=args.engine_file_path)
    de.extract_data()


if __name__ == '__main__':
    main()
