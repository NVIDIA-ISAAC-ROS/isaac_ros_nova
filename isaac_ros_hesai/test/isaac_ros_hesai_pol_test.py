# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2022-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
import pathlib
import subprocess
import time

from isaac_ros_test import IsaacROSBaseTest
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

import launch_testing
import pytest
import rclpy
from sensor_msgs.msg import PointCloud2


@pytest.mark.rostest
def generate_test_description():
    # Ping the lidar ip to check if lidar is present before trying to test the node
    command = 'ping -c 1 -w 2 192.168.1.201'
    result = subprocess.run(command, shell=True, capture_output=True, text=True)
    # 0 return code means response was received
    if(result.returncode == 0):
        IsaacROSHesaiPOLTest.skip_test = False
        """Generate launch description with all ROS 2 nodes for testing."""
        hesai_node = ComposableNode(
            package='isaac_ros_hesai',
            plugin='nvidia::isaac_ros::hesai::HesaiNode',
            name='hesai',
            namespace=IsaacROSHesaiPOLTest.generate_namespace()
        )

        hesai_container = ComposableNodeContainer(
            package='rclcpp_components',
            name='hesai_container',
            namespace='',
            executable='component_container_mt',
            composable_node_descriptions=[
                hesai_node,
            ],
            output='screen'
        )
        return IsaacROSHesaiPOLTest.generate_test_description([
            hesai_container
        ])
    else:
        IsaacROSHesaiPOLTest.skip_test = True
        return IsaacROSHesaiPOLTest.generate_test_description(
            [launch_testing.actions.ReadyToTest()])


class IsaacROSHesaiPOLTest(IsaacROSBaseTest):
    """Test for Isaac ROS Hesai Proof of Life."""

    filepath = pathlib.Path(os.path.dirname(__file__))

    def test_hesai_pipeline(self) -> None:
        if self.skip_test:
            self.skipTest('No lidar detected! Skipping test.')
        else:
            """Expect the pipeline to produce PointCloud2 data from Hesai lidar."""
            self.generate_namespace_lookup(
                ['pointcloud'])

            received_messages = {}
            pointcloud_sub, = self.create_logging_subscribers(
                [('pointcloud', PointCloud2)], received_messages)

            try:
                # Wait at most TIMEOUT seconds for subscriber to respond
                TIMEOUT = 20
                end_time = time.time() + TIMEOUT

                done = False
                while time.time() < end_time:
                    rclpy.spin_once(self.node, timeout_sec=0.1)

                    # If we have received exactly one message on the output topic, break
                    if 'pointcloud' in received_messages:
                        done = True
                        break

                self.assertTrue(
                    done, "Didn't receive output on pointcloud topic!")

                # Collect received detections
                pointcloud_received = received_messages['pointcloud']

                # Make sure that at least one detection was found
                self.assertGreater(len(pointcloud_received.data), 0,
                                   "Didn't find any data in the PointCloud2 message!")

            finally:
                pass
                self.assertTrue(self.node.destroy_subscription(pointcloud_sub))
