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

from datetime import datetime

from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, LogInfo, Shutdown, TimerAction
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    now = datetime.now().strftime('%Y_%m_%d-%H_%M_%S')
    datetime_str = "'_' + '{}'".format(now)

    recording_dir_arg  = DeclareLaunchArgument(
        'recording_dir',
        description='The directory in which to put the recording.',
        default_value='/mnt/nova_ssd/recordings'
    )

    rosbag_name_arg  = DeclareLaunchArgument(
        'rosbag_name',
        description='The name of the folder which the rosbag is stored in',
        default_value='rosbag2'
    )

    append_datetime_arg = DeclareLaunchArgument(
        'append_datetime',
        description='Flag to append date and time to the rosbag name',
        default_value='True'  # or 'False' depending on your default preference
    )

    rosbag_name = PythonExpression([
        "'", LaunchConfiguration('rosbag_name'), "' + (", datetime_str, " if ",
        LaunchConfiguration('append_datetime'), " else '')"
    ])

    output_path = PythonExpression(["'",
                                    LaunchConfiguration('recording_dir'),
                                    "' + '/' + '",
                                    rosbag_name, "'"])

    rosbag_node = TimerAction(
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'bag', 'record', '--storage', 'mcap', '--output', output_path,
                     '/rosout',
                     '/tf',
                     '/tf_static',
                     '/front_stereo_camera/left/image_compressed',
                     '/front_stereo_camera/left/camera_info',
                     '/front_stereo_camera/right/image_compressed',
                     '/front_stereo_camera/right/camera_info',
                     '/back_stereo_camera/left/image_compressed',
                     '/back_stereo_camera/left/camera_info',
                     '/back_stereo_camera/right/image_compressed',
                     '/back_stereo_camera/right/camera_info',
                     '/left_stereo_camera/left/image_compressed',
                     '/left_stereo_camera/left/camera_info',
                     '/left_stereo_camera/right/image_compressed',
                     '/left_stereo_camera/right/camera_info',
                     '/right_stereo_camera/left/image_compressed',
                     '/right_stereo_camera/left/camera_info',
                     '/right_stereo_camera/right/image_compressed',
                     '/right_stereo_camera/right/camera_info',
                     '/front_fisheye_camera/left/image_compressed',
                     '/front_fisheye_camera/left/camera_info',
                     '/back_fisheye_camera/left/image_compressed',
                     '/back_fisheye_camera/left/camera_info',
                     '/left_fisheye_camera/left/image_compressed',
                     '/left_fisheye_camera/left/camera_info',
                     '/right_fisheye_camera/left/image_compressed',
                     '/right_fisheye_camera/left/camera_info',
                     '/front_2d_lidar/scan',
                     '/back_2d_lidar/scan',
                     '/front_3d_lidar/lidar_packets',
                     '/front_stereo_imu/imu',
                     '/chassis/imu',
                     '/chassis/ticks',
                     '/chassis/odom',
                     '/chassis/battery_state'],
                output='screen',
                on_exit=Shutdown(),
            ),
            TimerAction(
                actions=[
                    ExecuteProcess(cmd=['cp', '-r', '/etc/nova', output_path]),
                    ExecuteProcess(cmd=['cp', '-r', '/tmp/hesai', output_path]),
                ],
                period=1.0,
            ),
            TimerAction(
                actions=[
                    LogInfo(msg='Recording started'),
                ],
                period=3.0,
            ),
        ],
        period=15.0,
    )

    return LaunchDescription([recording_dir_arg, rosbag_name_arg, append_datetime_arg, rosbag_node])

