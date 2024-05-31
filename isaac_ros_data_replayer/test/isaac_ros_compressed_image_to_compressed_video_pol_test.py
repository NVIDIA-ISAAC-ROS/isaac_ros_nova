# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2023-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

import time

from foxglove_msgs.msg import CompressedVideo
from isaac_ros_test import IsaacROSBaseTest
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import numpy as np
import pytest
import rclpy
from sensor_msgs.msg import CompressedImage


@pytest.mark.rostest
def generate_test_description():
    """
    Isaac ROS Compressed Image To Compressed Video Converter.

    This test publishes CompressedImages from and checks if
    the received CompressedVideos have the same data
    """
    compressed_image_to_compressed_video_node = ComposableNode(
        name='compressed_image_to_compressed_video_node',
        package='isaac_ros_data_replayer',
        plugin='nvidia::isaac_ros::data_replayer::CompressedImageToCompressedVideoNode',
        namespace=IsaacROSCompressedImageToCompressedVideoTest.generate_namespace(),
    )

    compressed_image_to_compressed_video_container = ComposableNodeContainer(
        package='rclcpp_components',
        name='compressed_image_to_compressed_video_container',
        namespace='',
        executable='component_container_mt',
        composable_node_descriptions=[
            compressed_image_to_compressed_video_node,
        ],
        output='screen',
    )
    return IsaacROSCompressedImageToCompressedVideoTest.generate_test_description(
        [compressed_image_to_compressed_video_container]
    )


class IsaacROSCompressedImageToCompressedVideoTest(IsaacROSBaseTest):
    """Test for Isaac ROS Compressed Image to Compressed Video Proof of Life."""

    def test_pointcloud_to_flatscan_pipeline(self) -> None:
        """Expect the pipeline to produce CompressedVideo data from a CompressedImage."""
        self.generate_namespace_lookup(['image_compressed', 'video_compressed'])

        compressed_image_pub = self.node.create_publisher(
            CompressedImage, self.namespaces['image_compressed'], self.DEFAULT_QOS
        )

        received_messages = {}
        (compressed_video_sub,) = self.create_logging_subscribers(
            [('video_compressed', CompressedVideo)], received_messages
        )
        try:
            # Wait at most TIMEOUT seconds for subscriber to respond
            TIMEOUT = 20
            end_time = time.time() + TIMEOUT

            compressed_img = CompressedImage()
            compressed_img.header.frame_id = 'camera'
            compressed_img.header.stamp = self.node.get_clock().now().to_msg()
            compressed_img.format = 'h264'
            compressed_img.data = np.random.randint(
                255, size=(640, 480, 3), dtype=np.uint8
            ).tobytes()

            done = False
            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=0.1)
                # If we have received exactly one message on the output topic, break
                compressed_image_pub.publish(compressed_img)
                if 'video_compressed' in received_messages:
                    done = True
                    break

            self.assertTrue(done, 'Did not receive output on video_compressed topic!')
            compressed_video = received_messages['video_compressed']
            self.assertEqual(compressed_video.frame_id, compressed_img.header.frame_id)
            self.assertEqual(compressed_video.timestamp, compressed_img.header.stamp)
            self.assertEqual(compressed_video.format, compressed_img.format)
            self.assertImagesEqual(
                np.asarray(compressed_video.data), np.asarray(compressed_img.data)
            )

        finally:
            self.assertTrue(self.node.destroy_subscription(compressed_video_sub))
            self.assertTrue(self.node.destroy_publisher(compressed_image_pub))
