# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

from isaac_ros_nova_interfaces.msg import CorrelatedTimestamp
from isaac_ros_test import IsaacROSBaseTest
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

import launch_testing
import pytest
import rclpy


@pytest.mark.rostest
def generate_test_description():
    # Ping the nvpps device to check if present before trying to test the node
    command = 'ls /dev/nvpps0'
    result = subprocess.run(command, shell=True, capture_output=True, text=True)
    # 0 return code means response was received
    if(result.returncode == 0):
        IsaacROSCorrelatedTimestampDriverPOLTest.skip_test = False
        """Generate launch description with all ROS 2 nodes for testing."""
        correlated_timestamp_driver_node = ComposableNode(
            package='isaac_ros_correlated_timestamp_driver',
            plugin='nvidia::isaac_ros::correlated_timestamp_driver::CorrelatedTimestampDriverNode',
            name='correlated_timestamp_driver',
            namespace=IsaacROSCorrelatedTimestampDriverPOLTest.generate_namespace()
        )

        correlated_timestamp_driver_container = ComposableNodeContainer(
            package='rclcpp_components',
            name='correlated_timestamp_driver_container',
            namespace='',
            executable='component_container_mt',
            composable_node_descriptions=[
                correlated_timestamp_driver_node,
            ],
            output='screen'
        )
        return IsaacROSCorrelatedTimestampDriverPOLTest.generate_test_description([
            correlated_timestamp_driver_container
        ])
    else:
        IsaacROSCorrelatedTimestampDriverPOLTest.skip_test = True
        return IsaacROSCorrelatedTimestampDriverPOLTest.generate_test_description(
            [launch_testing.actions.ReadyToTest()])


class IsaacROSCorrelatedTimestampDriverPOLTest(IsaacROSBaseTest):
    """Test for Isaac ROS Timestamp Correlator Driver Proof of Life."""

    filepath = pathlib.Path(os.path.dirname(__file__))

    def test_correlated_timestamp_driver_pipeline(self) -> None:
        if self.skip_test:
            self.skipTest('No nvpps device detected! Skipping test.')
        else:
            """Expect the pipeline to produce CorrelatedTimestamp data."""
            self.generate_namespace_lookup(
                ['correlated_timestamp'])

            received_messages = {}
            correlated_timestamp_sub, = self.create_logging_subscribers(
                [('correlated_timestamp', CorrelatedTimestamp)], received_messages)

            try:
                # Wait at most TIMEOUT seconds for subscriber to respond
                TIMEOUT = 20
                end_time = time.time() + TIMEOUT

                done = False
                while time.time() < end_time:
                    rclpy.spin_once(self.node, timeout_sec=0.1)

                    # If we have received exactly one message on the output topic, break
                    if 'correlated_timestamp' in received_messages:
                        done = True
                        break

                self.assertTrue(
                    done, "Didn't receive output on correlated_timestamp topic!")

            finally:
                pass
                self.assertTrue(self.node.destroy_subscription(correlated_timestamp_sub))
