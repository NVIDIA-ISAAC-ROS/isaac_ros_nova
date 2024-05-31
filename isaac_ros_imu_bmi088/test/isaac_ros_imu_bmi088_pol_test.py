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
from sensor_msgs.msg import Imu


@pytest.mark.rostest
def generate_test_description():
    # Check if accelerometer and gyroscoper exists
    # If the device0 name is 'accelerometer\n' that means that the accelerometer exists
    # If the device1 name is 'gyroscope\n' that means that the gyroscope exists
    command_accel = r'cat /sys/bus/iio/devices/iio\:device0/name'
    result_accel = subprocess.run(command_accel, shell=True, capture_output=True, text=True)
    command_gyro = r'cat /sys/bus/iio/devices/iio\:device1/name'
    result_gyro = subprocess.run(command_gyro, shell=True, capture_output=True, text=True)
    if(result_accel.stdout == 'accelerometer\n' and result_gyro.stdout == 'gyroscope\n'):
        IsaacROSBmi088POLTest.skip_test = False
        """Generate launch description with all ROS 2 nodes for testing."""
        correlated_timestamp_driver_node = ComposableNode(
            name='correlated_timestamp_driver_node',
            package='isaac_ros_correlated_timestamp_driver',
            plugin='nvidia::isaac_ros::correlated_timestamp_driver::CorrelatedTimestampDriverNode',
            namespace=IsaacROSBmi088POLTest.generate_namespace(),
            parameters=[{'use_time_since_epoch': False,
                        'nvpps_dev_file': '/dev/nvpps0'}])

        imu_bmi088_node = ComposableNode(
            package='isaac_ros_imu_bmi088',
            plugin='nvidia::isaac_ros::imu_bmi088::Bmi088Node',
            name='bmi088',
            namespace=IsaacROSBmi088POLTest.generate_namespace()
        )

        bmi088_container = ComposableNodeContainer(
            package='rclcpp_components',
            name='bmi088_container',
            namespace='',
            executable='component_container_mt',
            composable_node_descriptions=[
                correlated_timestamp_driver_node,
                imu_bmi088_node
            ],
            output='screen'
        )
        return IsaacROSBmi088POLTest.generate_test_description([
            bmi088_container
        ])
    else:
        IsaacROSBmi088POLTest.skip_test = True
        return IsaacROSBmi088POLTest.generate_test_description(
            [launch_testing.actions.ReadyToTest()])


class IsaacROSBmi088POLTest(IsaacROSBaseTest):
    """Test for Isaac ROS Bmi088 Proof of Life."""

    filepath = pathlib.Path(os.path.dirname(__file__))

    def test_bmi088_pipeline(self) -> None:
        if self.skip_test:
            self.skipTest('No lidar detected! Skipping test.')
        else:
            """Expect the pipeline to produce Imu data from Bmi088 lidar."""
            self.generate_namespace_lookup(
                ['imu'])

            received_messages = {}
            bmi088_sub, = self.create_logging_subscribers(
                [('imu', Imu)], received_messages)

            try:
                # Wait at most TIMEOUT seconds for subscriber to respond
                TIMEOUT = 20
                end_time = time.time() + TIMEOUT

                done = False
                while time.time() < end_time:
                    rclpy.spin_once(self.node, timeout_sec=0.1)

                    # If we have received exactly one message on the output topic, break
                    if 'imu' in received_messages:
                        done = True
                        break

                # Check if atleast one msg was received on the output imu topic
                self.assertTrue(
                    done, "Didn't receive output on imu topic!")

            finally:
                pass
                self.assertTrue(self.node.destroy_subscription(bmi088_sub))
