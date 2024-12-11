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

import pathlib

from diagnostic_msgs.msg import DiagnosticArray
from hesai_ros_driver.msg import UdpFrame
import isaac_ros_launch_utils as lu
from isaac_ros_nova_interfaces.msg import CorrelatedTimestamp
from isaac_ros_test import IsaacROSBaseTest
from launch import LaunchDescription
import launch_testing
import pytest
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2
import yaml


@pytest.mark.rostest
def generate_test_description():
    # Check if the system info file exists
    system_info_path = pathlib.Path('/etc/nova/systeminfo.yaml')
    if system_info_path.exists():
        IsaacROSNovaLaunchTest.skip_test = False
        actions = []
        actions.append(
            lu.include('isaac_ros_nova', 'launch/nova.launch.py'))
        # Required for ROS launch testing.
        actions.append(launch_testing.util.KeepAliveProc())
        actions.append(launch_testing.actions.ReadyToTest())
        return LaunchDescription(actions)
    else:
        # TODO (sgillen) can we make pytest.skip work with colcon? isaac_ros_argus_camera uses
        # a different method, so use that here to be consitent.
        # pytest.skip(f"System info file does not exist: {system_info_path}")
        IsaacROSNovaLaunchTest.skip_test = True
        return IsaacROSNovaLaunchTest.generate_test_description(
            [launch_testing.actions.ReadyToTest()])


class IsaacROSNovaLaunchTest(IsaacROSBaseTest):
    filepath = pathlib.Path(__file__).parent
    skip_test = False

    def test_nova_launch(self):
        """
        Test for nova.launch proof of life.

        Checks if the launch file can be run and all the expected topics are published.
        """
        if self.skip_test:
            self.skipTest('No camera detected! Skipping test.')
        else:
            TIMEOUT = 30
            received_messages = {}

            # Load the sensor configuration from the YAML file
            with open('/etc/nova/systeminfo.yaml', 'r') as file:
                system_info = yaml.safe_load(file)

            sensors = system_info.get('sensors', {})

            # Start with topics that we expect to always be present
            topics = [
                ('/diagnostics', DiagnosticArray),
                ('/diagnostics_agg', DiagnosticArray)
            ]

            # Add camera topics based on the sensor configuration
            for sensor_name, sensor_info in sensors.items():
                if 'fisheye_camera' in sensor_name:
                    topics.append((f'/{sensor_name}/left/camera_info', CameraInfo))
                    topics.append((f'/{sensor_name}/left/image_raw', Image))
                elif 'stereo_camera' in sensor_name:
                    topics.append((f'/{sensor_name}/left/camera_info', CameraInfo))
                    topics.append((f'/{sensor_name}/left/image_raw', Image))
                    topics.append((f'/{sensor_name}/right/camera_info', CameraInfo))
                    topics.append((f'/{sensor_name}/right/image_raw', Image))
                elif sensor_info['type'] == 'hesai':
                    topics.append((f'/{sensor_name}/lidar_packets', UdpFrame))
                    topics.append((f'/{sensor_name}/lidar_points', PointCloud2))
                # TODO (sgillen) add support for generic 3d lidar if that is ever needed
                # TODO (sgillen) add support for 2d lidar
                elif 'imu' in sensor_name:
                    topics.append((f'/{sensor_name}/imu', Imu))

            # Add correlated timestamp topic if there are any cameras
            if any('camera' in sensor_name for sensor_name in sensors):
                topics.append(('/correlated_timestamp', CorrelatedTimestamp))

            # Create subscribers for the topics
            subs = self.create_logging_subscribers(
                topics,
                received_messages,
                use_namespace_lookup=False,
                accept_multiple_messages=True
            )
            try:
                print(__file__ + ': Spinning')
                self.spin_node_until_messages_received(received_messages, TIMEOUT)
                print(__file__ + ': Spinning completed')
                self.assert_messages_received(received_messages)

            finally:
                [self.node.destroy_subscription(sub) for sub in subs]
