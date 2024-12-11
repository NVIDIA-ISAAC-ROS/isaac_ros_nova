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

import boto3
import os

from isaac_ros_nova_recorder.action import UploadRosbag
from isaac_ros_nova_recorder.msg import TransferStatus
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node


class RosbagUploaderNode(Node):

    class Callback:

        def __init__(self, action, msg):
            self.action = action
            self.msg = msg

        def __call__(self, bytes_transferred):
            self.msg.bytes_transferred += bytes_transferred
            if self.msg.bytes_transferred == self.msg.bytes_total_size:
                self.msg.status = 'COMPLETED'
                self.action.succeed()
            self.action.publish_feedback(UploadRosbag.Feedback(status=self.msg))

    def __init__(self):
        super().__init__('rosbag_uploader')

        self.declare_parameter('directory', '.')
        self.directory = self.get_parameter('directory').get_parameter_value().string_value

        self.declare_parameter('bucket', '')
        self.bucket = self.get_parameter('bucket').get_parameter_value().string_value

        self.action_server = ActionServer(
            node=self,
            action_type=UploadRosbag,
            action_name='upload_rosbag',
            execute_callback=self.upload_rosbag,
            cancel_callback=self.cancel_upload,
        )

        self.s3 = boto3.client('s3')

    def upload_rosbag(self, action):
        rosbag_path = os.path.join(self.directory, action.request.rosbag)
        if self.bucket and os.path.exists(rosbag_path) and os.path.isdir(rosbag_path):
            msg = TransferStatus()
            msg.rosbag = action.request.rosbag
            msg.status = 'IN_PROGRESS'
            msg.bytes_total_size = 0
            msg.bytes_transferred = 0

            file_paths = []
            for path, directories, files in os.walk(rosbag_path):
                if path == rosbag_path:
                    for file in files:
                        if file == 'metadata.yaml' or file.endswith('.mcap'):
                            file_path = os.path.join(path, file)
                            file_paths.append(file_path)
                            msg.bytes_total_size += os.path.getsize(file_path)

            self.get_logger().info(f'Uploading {action.request.rosbag} to {self.bucket}')
            for file_path in file_paths:
                self.s3.upload_file(
                    Filename=file_path,
                    Bucket=self.bucket,
                    Key=os.path.join(action.request.rosbag, os.path.basename(file_path)),
                    Callback=RosbagUploaderNode.Callback(action, msg)
                )

            self.get_logger().info(f'Uploaded {action.request.rosbag} to {self.bucket}')
            result = UploadRosbag.Result(success=True)
        else:
            if not self.bucket:
                self.get_logger().warning('S3 bucket is not configured')
            if not os.path.exists(rosbag_path):
                self.get_logger().warning(f'{action.request.rosbag} does not exist')
            if not os.path.isdir(rosbag_path):
                self.get_logger().warning(f'{action.request.rosbag} is not a rosbag')
            result = UploadRosbag.Result(success=False)
        return result

    def cancel_upload(self):
        self.get_logger().warning('TODO(ayusmans): cancel_upload()')


def main():
    rclpy.init()
    rclpy.spin(RosbagUploaderNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
