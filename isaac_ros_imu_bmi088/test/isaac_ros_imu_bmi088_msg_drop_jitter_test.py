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

EXPECTED_FPS = 200
EXPECTED_ACQ_TIME_DT = 1/EXPECTED_FPS
# Percentage of allowed dropped frames calculated from
# EXPECTED_FPS and TIMEOUT
MSG_DROP_TOL_PERCENT = 0.01
# Jitter tolerance calculated as
# percentage of EXPECTED_ACQ_TIME_DT
FPS_DT_TOL_SECS_PERCENT = 0.1
# Allowed difference between wall clock time dt
# and acq time dt
ALLOWED_DIFF_WALL_CLOCK_ACQ_TIME = 0.05
VERBOSE = False


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
        IsaacROSBmi088MsgDropJitterTest.skip_test = False
        """Generate launch description with all ROS 2 nodes for testing."""
        correlated_timestamp_driver_node = ComposableNode(
            name='correlated_timestamp_driver_node',
            package='isaac_ros_correlated_timestamp_driver',
            plugin='nvidia::isaac_ros::correlated_timestamp_driver::CorrelatedTimestampDriverNode',
            namespace=IsaacROSBmi088MsgDropJitterTest.generate_namespace(),
            parameters=[{'use_time_since_epoch': False,
                        'nvpps_dev_file': '/dev/nvpps0'}])

        imu_bmi088_node = ComposableNode(
            package='isaac_ros_imu_bmi088',
            plugin='nvidia::isaac_ros::imu_bmi088::Bmi088Node',
            name='bmi088',
            namespace=IsaacROSBmi088MsgDropJitterTest.generate_namespace()
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
        return IsaacROSBmi088MsgDropJitterTest.generate_test_description([
            bmi088_container
        ])
    else:
        IsaacROSBmi088MsgDropJitterTest.skip_test = True
        return IsaacROSBmi088MsgDropJitterTest.generate_test_description(
            [launch_testing.actions.ReadyToTest()])


class IsaacROSBmi088MsgDropJitterTest(IsaacROSBaseTest):
    """Test for Isaac ROS Bmi088 Msg Drop and Jitter."""

    filepath = pathlib.Path(os.path.dirname(__file__))

    def test_bmi088_pipeline(self) -> None:
        if self.skip_test:
            self.skipTest('No lidar detected! Skipping test.')
        else:
            """Expect the number of msgs dropped and jitter in acquisition timestamps
                between the messages to be within threshold for Imu data from Bmi088 lidar."""
            self.generate_namespace_lookup(
                ['imu'])

            received_messages = {}
            bmi088_sub, = self.create_logging_subscribers(
                [('imu', Imu)], received_messages, accept_multiple_messages=True)

            try:
                # Run for TIMEOUT seconds
                TIMEOUT = 2
                end_time = time.time() + TIMEOUT

                while time.time() < end_time:
                    rclpy.spin_once(self.node, timeout_sec=0.1)

                imu_msgs = received_messages['imu']
                # Calculate the time between first and last msgs' acquire timestamp
                first_imu_msg = imu_msgs[0].header.stamp
                last_imu_msg = imu_msgs[-1].header.stamp
                # Number of seconds between frist and last received imu msg
                acq_time_dt = (last_imu_msg.sec-first_imu_msg.sec) + \
                    1e-9*(last_imu_msg.nanosec-first_imu_msg.nanosec)
                # Check if the difference between first and last msg using
                #  wall clock time and acq time is within tolerance
                if(VERBOSE):
                    print('abs(TIMEOUT-acq_time_dt) : ', abs(TIMEOUT-acq_time_dt))
                    print('ALLOWED_DIFF_WALL_CLOCK_ACQ_TIME : ', ALLOWED_DIFF_WALL_CLOCK_ACQ_TIME)
                    print('\n')
                self.assertLess(abs(TIMEOUT-acq_time_dt),
                                ALLOWED_DIFF_WALL_CLOCK_ACQ_TIME)
                # Calculate expecte05 number of msgs based on the above time interval
                # assuming the fps to be EXPECTED_FPS
                expected_num_msgs = acq_time_dt * EXPECTED_FPS
                actual_num_msgs = len(imu_msgs)
                if(VERBOSE):
                    print('acq_time_dt(secs) : ', acq_time_dt)
                    print('expected_num_msgs : ', expected_num_msgs)
                    print('actual_num_msgs : ', actual_num_msgs)
                    print('diff : ', abs(expected_num_msgs-actual_num_msgs))
                    print('max allowed diff : ', expected_num_msgs*MSG_DROP_TOL_PERCENT)
                    print('\n')
                # Check if the difference between expected and received msgs are within threshold
                self.assertLess(abs(expected_num_msgs-actual_num_msgs),
                                expected_num_msgs*MSG_DROP_TOL_PERCENT)

                # We need atleast two msgs to check for jitter
                self.assertGreaterEqual(actual_num_msgs, 2)
                max_jitter = 0
                prev_msg_acq_time = imu_msgs[0].header.stamp
                for imu_msg in imu_msgs[1:]:
                    curr_msg_acq_time = imu_msg.header.stamp
                    actual_acq_time_dt = (curr_msg_acq_time.sec-prev_msg_acq_time.sec) + \
                        1e-9*(curr_msg_acq_time.nanosec-prev_msg_acq_time.nanosec)
                    prev_msg_acq_time = imu_msg.header.stamp
                    max_jitter = max(max_jitter, abs(EXPECTED_ACQ_TIME_DT-actual_acq_time_dt))
                    if(VERBOSE):
                        print('EXPECTED_ACQ_TIME_DT(secs) : ', EXPECTED_ACQ_TIME_DT)
                        print('actual_acq_time_dt(secs) : ', actual_acq_time_dt)
                    self.assertLess(abs(EXPECTED_ACQ_TIME_DT-actual_acq_time_dt),
                                    EXPECTED_ACQ_TIME_DT*FPS_DT_TOL_SECS_PERCENT)
                if(VERBOSE):
                    print('max_jitter(secs) : ', max_jitter)

            finally:
                pass
                self.assertTrue(self.node.destroy_subscription(bmi088_sub))
